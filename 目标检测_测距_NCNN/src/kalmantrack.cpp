#include <cmath>
#include <algorithm>
#include "../include/objdetect.h"
#include "../include/kalmantrack.h"

int KalmanTrack::obj_num_ = 0;

KalmanBox convert_box_to_x(const cv::Rect_<float> rect)
{
    KalmanBox ret;
    ret.x = rect.x + rect.width / 2;
    ret.y = rect.y + rect.height / 2;
    ret.s = rect.width * rect.height;
    ret.r = rect.width / rect.height;
    return ret;
}

cv::Rect_<float> convert_x_to_box(const Eigen::VectorXf x_)
{
    cv::Rect_<float> ret;
    ret.width = (float)std::sqrt((double)x_[2] * (double)x_[3]);
    ret.height = x_[2] / ret.width;
    ret.x = x_[0] - ret.width / 2;
    ret.y = x_[1] - ret.height / 2;
}

float cal_iou(cv::Rect_<float> box1, cv::Rect_<float> box2)
{
    float s1, s2;
    float x1min, y1min, x1max, y1max;
    float x2min, y2min, x2max, y2max;

    x1min = box1.x;
    y1min = box1.y;
    x1max = x1min + box1.width;
    y1max = y1min + box1.height;

    x2min = box2.x;
    y2min = box2.y;
    x2max = x2min + box2.width;
    y2max = y2min + box2.height;

    // 计算两框的面积
    s1 = (y1max - y1min + 1.) * (x1max - x1min + 1.);
    s2 = (y2max - y2min + 1.) * (x2max - x2min + 1.);

    float inter_h, inter_w;
    float xmin, ymin, xmax, ymax;

    xmin = std::max(x1min,x2min);
    ymin = std::max(y1min,y2min);
    xmax = std::min(x1max,x2max);
    ymax = std::min(y1max,y2max);
    inter_h = std::max(ymax - ymin + 1, (float)0.000001);
    inter_w = std::max(xmax - xmin + 1, (float)0.000001);

    float intersection = inter_h * inter_w;
    float all = s1 + s2 - intersection;

    return intersection / all;
}

void associate_detections_to_trackers(std::vector<Object> dets, 
std::vector<KalmanTrack> trackers, std::vector<std::pair<int,int> >& matched,
std::vector<int>& unmatched_dets, float iou_threshold)
{
    if(trackers.size() == 0)
    {
        for(int i = 0; i < dets.size(); i++)
        {
            unmatched_dets.push_back(i);
        }
        return;
    }
    // 计算iou矩阵
    std::vector<std::vector<float> > iou_matrix(0);
    std::vector<float> iou_matrix_row(0);
    for(int i = 0; i < dets.size(); i++)
    {
        for(int j = 0; j < trackers.size(); j++)
        {
            float iou = cal_iou(dets[i].rect, trackers[j].getbox());
            iou_matrix_row.push_back(iou);
        }
        iou_matrix.push_back(iou_matrix_row);
        std::vector<float>().swap(iou_matrix_row);
    }
    

}

KalmanTrack::KalmanTrack(Object& obj)
{
    ID_ = obj_num_;
    obj_num_++;
    obj.ID = ID_;
    pre_num_ = 0;
    update_num_ = 0;
    pre_ctn_num_ = 0;
    update_ctn_num_ = 0;

    KalmanBox kalbox = convert_box_to_x(obj.rect);
    x_ = Eigen::VectorXf(7);
    F_ = Eigen::MatrixXf(7, 7);
    P_ = Eigen::MatrixXf(7, 7);
    Q_ = Eigen::MatrixXf(7, 7);
    H_ = Eigen::MatrixXf(4, 7);
    R_ = Eigen::MatrixXf(4, 4);
    K_ = Eigen::MatrixXf(7, 4);

    x_ << kalbox.x, kalbox.y, kalbox.s, kalbox.r, 0, 0, 0;
    F_ << 1,0,0,0,1,0,0,
            0,1,0,0,0,1,0,
            0,0,1,0,0,0,1,
            0,0,0,1,0,0,0,
            0,0,0,0,1,0,0,
            0,0,0,0,0,1,0,
            0,0,0,0,0,0,1;
    P_ << 10,0,0,0,0,0,0,
            0,10,0,0,0,0,0,
            0,0,10,0,0,0,0,
            0,0,0,10,0,0,0,
            0,0,0,0,10000,0,0,
            0,0,0,0,0,10000,0,
            0,0,0,0,0,0,10000;
    Q_ << 1,0,0,0,0,0,0,
            0,1,0,0,0,0,0,
            0,0,1,0,0,0,0,
            0,0,0,1,0,0,0,
            0,0,0,0,0.001,0,0,
            0,0,0,0,0,0.001,0,
            0,0,0,0,0,0,0.0001;
    H_ << 1,0,0,0,0,0,0,
            0,1,0,0,0,0,0,
            0,0,1,0,0,0,0,
            0,0,0,1,0,0,0;
    R_ << 1,0,0,0,
            0,1,0,0,
            0,0,10,0,
            0,0,0,10;
    I_ = Eigen::MatrixXf::Identity(7, 7);
}

void KalmanTrack::predict()
{
    // 面积恒大于0
    if(x_[6] + x_[2] <= 0.)
    {
        x_[6] = 0;
    }

    /* 卡尔曼预测 */
    x_ = F_ * x_;
    Eigen::MatrixXf Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
    Eigen::MatrixXf S = H_ * P_ * H_.transpose() + R_;
    K_ = P_ * H_.transpose() * S.inverse();

    pre_num_ += 1;
    if(pre_ctn_num_ > 0)
    {
        update_ctn_num_ = 0;
    }
    pre_ctn_num_ += 1;

    return;
}

void KalmanTrack::update(const cv::Rect_<float> box)
{
    pre_ctn_num_ = 0;
    update_ctn_num_ += 1;

    /* 卡尔曼更新 */
    KalmanBox kalbox = convert_box_to_x(box);
    Eigen::VectorXf z(4);
    z << kalbox.x, kalbox.y, kalbox.s, kalbox.r;
    Eigen::VectorXf y = z - H_ * x_;
    x_ = x_ + (K_ * y);
    P_ = (I_ - K_ * H_) * P_;

    update_num_ += 1;
    return;
}

cv::Rect_<float> KalmanTrack::getbox()
{
    return convert_x_to_box(x_);
}

Sort::Sort()
{
    frame_count_ = 0;
    max_pre_num_ = 3;
    min_update_num_ = 3;
    iou_threshold_ = 0.3;
}

std::vector<Object> Sort::update(std::vector<Object> objs)
{
    frame_count_ += 1;
    // 每个目标进行预测
    for(int i = 0; i < trackers_.size(); i++)
    {
        trackers_[i].predict();
    }
    std::vector<int> unmatched_dets(0);
    std::vector<std::pair<int,int> > matched(0);


}

