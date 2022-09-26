#pragma once

#include <time.h>
#include "Eigen/Dense"
#include "../include/objdetect.h"

struct KalmanBox
{
    float x;
    float y;
    float s;
    float r;
};

class KalmanTrack{
private:
    static int obj_num_;            // 记录卡尔曼滤波器个数，用于分配目标ID
    /* 跟踪参数 */
    int ID_;                        // 每个目标的ID
    int pre_num_;                   // 预测总次数
    int update_num_;                // 更新总次数
    int pre_ctn_num_;               // 连续预测次数
    int update_ctn_num_;            // 连续更新次数
    /* 卡尔曼滤波器参数 */
    Eigen::VectorXf x_;             // 状态向量[x,y,s,r,x_,y_,s_]
    Eigen::MatrixXf F_;             // 状态转移矩阵
    Eigen::MatrixXf P_;             // 状态误差矩阵
    Eigen::MatrixXf Q_;             // 状态噪声矩阵
    Eigen::MatrixXf H_;             // 状态观测矩阵
    Eigen::MatrixXf R_;             // 状态噪声矩阵
    Eigen::MatrixXf K_;             // 卡尔曼增益
    Eigen::MatrixXf I_;             // 单位矩阵

public:
    KalmanTrack(Object& obj);
    void predict();
    void update(const cv::Rect_<float> box);
    cv::Rect_<float> getbox();
};

class Sort{
private:
    int frame_count_;                   // 总帧数
    int max_pre_num_;                   // 最大连续预测次数
    int min_update_num_;                // 最小连续更新次数
    int iou_threshold_;                 // 检测框iou阈值
    std::vector<KalmanTrack> trackers_; // 多个卡尔曼滤波器

public:
    Sort();
    std::vector<Object> update(std::vector<Object> objs);

};

KalmanBox convert_box_to_x(const cv::Rect_<float> rect);
cv::Rect_<float> convert_x_to_box(const Eigen::VectorXf x_);

float cal_iou(cv::Rect_<float> box1, cv::Rect_<float> box2);
void associate_detections_to_trackers(std::vector<Object> dets, 
std::vector<KalmanTrack> trackers, std::vector<std::pair<int,int> >& matched,
std::vector<int>& unmatched_dets);
