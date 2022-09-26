#pragma once

#include "net.h"
#include "layer.h"

#include <time.h>
#include <vector>
#include <string>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define _SHOW_TIME_ (1)

struct Object
{
    int ID;
    int label;
    float prob;
    float disx;
    float disy;
    cv::Rect_<float> rect;  //检测框左上角坐标(x,y),宽度(w,h)
};

struct CameraCfg
{
    float height;
    float vx, vy;
    float fx, fy, cx, cy;
    cv::Mat CamYamMat, CamPitchMat, CamIntrinsicMatI;
};

class ObjTrack
{
private:
    /* 成员变量 */
    const int MAX_STRIDE_ = 64;
    const int target_size_ = 640;
    const float norm_vals_[3] = {1 / 255.f, 1 / 255.f, 1 / 255.f};
    const float nms_threshold_ = 0.45f;
    const float prob_threshold_ = 0.25f;

    ncnn::Mat ncnnMatIN_;
    ncnn::Net ncnnyolov5_;
    ncnn::Mat ncnnMatOUT_;
    ncnn::Mat ncnnMatINPad_;
    ncnn::Mat ncnnanchors8_;
    ncnn::Mat ncnnanchors16_;
    ncnn::Mat ncnnanchors32_;

    /* 成员函数 */
    float sigmoid(float x);
    float intersection_area(const Object& a, const Object& b);
    void qsort_descent_inplace(std::vector<Object>& faceobjects);
    void qsort_descent_inplace(std::vector<Object>& faceobjects, int left, int right);
    void nms_sorted_bboxes(const std::vector<Object>& faceobjects, std::vector<int>& picked, float nms_threshold);
    void generate_proposals(const ncnn::Mat& anchors, int stride, const ncnn::Mat& in_pad, const ncnn::Mat& feat_blob, float prob_threshold, std::vector<Object>& objects);

public:
    /* 成员变量 */
    std::vector<Object> objects_;

    /* 成员函数 */
    ObjTrack();
    ~ObjTrack();
    int detect_yolov5(const cv::Mat& cvMatImg);

};

void draw_objects(const cv::Mat& bgr, const std::vector<Object>& objects, int id);