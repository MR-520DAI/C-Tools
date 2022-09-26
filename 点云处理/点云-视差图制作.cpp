#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/calib3d.hpp"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace cv;
using namespace pcl;

Mat GetDispImg(PointCloud<PointXYZ>::Ptr cloud, Mat mapx, Mat mapy, float fx, float base, Size img_size,
    Mat world2cam_T, Mat cameraMatrixL, Mat rectify_R);

int main()
{
    Size img_size(1280, 720);
    /* 相机参数 */
    // fx代表相机的x方向的归一化焦距
    float fx = 3.54136417e+03;
    // base代表左右相机x方向光心的间距
    float base = 0.175908760953822;
    // cameraMatrixL代表相机初始内参矩阵
    Mat cameraMatrixL = (Mat_<double>(3, 3) << 3.5204932328875097e+03, 0., 6.7804628181009662e+02, 0.,
        2.9619494807634046e+03, 3.6969187268740455e+02, 0., 0., 1.);
    // distCoeffL代表相机的畸变系数
    Mat distCoeffL = (Mat_<double>(5, 1) << -6.2448614987403028e-01, 2.6865938710745505e+00,
        -6.1227675920326821e-04, 1.7426909809930131e-04, 0.);
    // camera_Rl代表双目相机极线校正后的旋转矩阵
    Mat camera_Rl = (Mat_<double>(3, 3) << 0.99953187, 0.00557546, 0.03008233,
        -0.00579386, 0.99995745, 0.00717794,
        -0.03004103, -0.00734887, 0.99952165);
    // camera_Pl代表相机校正后的内参矩阵
    Mat camera_Pl = (Mat_<double>(3, 4) << 3.54136417e+03, 0.00000000e+00, 5.79143124e+02, 0.00000000e+00,
        0.00000000e+00, 3.54136417e+03, 3.66899548e+02, 0.00000000e+00,
        0.00000000e+00, 0.00000000e+00, 1.00000000e+00, 0.00000000e+00);
    // T代表激光雷达与左相机的变换矩阵；[R,t
    //                                   0,1] 
    Mat T = (Mat_<double>(4, 4) << -0.039858, -0.998975, 0.0214379, 0.0986614,
        -0.0201515, -0.020647, -0.999584, 0.117984,
        0.999002, -0.0402734, -0.0193079, 0.0560036,
        0, 0, 0, 1);
    // K代表相机初始内参矩阵，只是写成了齐次形式，方便后续计算
    Mat K = (Mat_<double>(3, 4) << 3.5204932328875097e+03, 0., 6.7804628181009662e+02, 0,
        0, 2.9619494807634046e+03, 3.6969187268740455e+02, 0,
        0, 0, 1, 0);
    // rectify_R代表双目相机极线校正后的旋转矩阵，只是写成了齐次形式，方便后续计算
    Mat rectify_R = (Mat_<double>(4, 4) << 0.99953187, 0.00557546, 0.03008233, 0,
        -0.00579386, 0.99995745, 0.00717794, 0,
        -0.03004103, -0.00734887, 0.99952165, 0);

    // 计算校正映射矩阵
    Mat mapxl, mapyl;
    initUndistortRectifyMap(cameraMatrixL, distCoeffL, camera_Rl, camera_Pl, img_size, CV_32FC1, mapxl, mapyl);

    char pcdname[256];
    char dispname[256];
    for (int i = 0; i < 3; i++)
    {
        sprintf(pcdname, "./pcd/%d.pcd", i);
        sprintf(dispname, "./disp_img/%d.png", i);

        /* 点云数据读取 */
        PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdname, *cloud) == -1)     // 打开点云文件
        {
            PCL_ERROR("Couldn't read file test_pcd.pcd\n");
            return(-1);
        }
        Mat img_disp = GetDispImg(cloud, mapxl, mapyl, fx, base, img_size, T, K, rectify_R);
        imwrite(dispname, img_disp);
        cout << "处理完毕:./disp_img" << i << ".png" << endl;
    }

    return 0;
}

Mat GetDispImg(PointCloud<PointXYZ>::Ptr cloud, Mat mapx, Mat mapy, float fx, float base, Size img_size,
    Mat world2cam_T, Mat cameraMatrixL, Mat rectify_R)
{
    Mat img_disp = Mat::zeros(img_size, CV_16UC1);
    Mat img_depth = Mat::zeros(img_size, CV_32FC1);
    
    for (int i = 0; i < cloud->size(); i++)
    {
        /* 点云处理 */
        Mat world_point = (Mat_<double>(4, 1) << (double)cloud->points[i].x, (double)cloud->points[i].y, (double)cloud->points[i].z, 1);
        // 激光（世界）坐标系（world_point）左乘相机外参、内参矩阵，转换到像素坐标系下（img_point）
        Mat img_point = cameraMatrixL * world2cam_T * world_point;
        // 双目相机还需极线校正（发生了旋转），因此实际深度需要左乘相机外参以及校正旋转矩阵
        Mat cam_point = world2cam_T * world_point;
        Mat rectify_point = rectify_R * cam_point;
        int x = (int)(img_point.at<double>(0, 0) / img_point.at<double>(2, 0));
        int y = (int)(img_point.at<double>(1, 0) / img_point.at<double>(2, 0));

        /* 深度图处理，将点云数据映射到图像中 */
        if (x >= 0 && x < img_size.width && y >= 0 && y < img_size.height)
        {
            if (img_depth.at<float>(y, x) == 0)
            {
                img_depth.at<float>(y, x) = (float)rectify_point.at<double>(2, 0);
            }
            else
            {
                if (img_depth.at<float>(y, x) > (float)rectify_point.at<double>(2, 0))
                {
                    img_depth.at<float>(y, x) = (float)rectify_point.at<double>(2, 0);
                }
            }
        }
    }

    /* 校正深度图 */
    for (int i = 0; i < img_size.height; i++)
    {
        for (int j = 0; j < img_size.width; j++)
        {
            int x = round(mapx.at<float>(i, j));
            int y = round(mapy.at<float>(i, j));
            if (x < 0 || x > img_size.width || y < 0 || y > img_size.height)
                continue;
            if (img_depth.at<float>(y, x) == 0)
                continue;
            if (img_disp.at<unsigned short>(i, j) == 0)
            {
                // 视差扩大64倍，模型训练所需
                unsigned short disp = (unsigned short)(fx * base * 64 / img_depth.at<float>(y, x));
                img_disp.at<unsigned short>(i, j) = disp;
            }
            else
            {
                unsigned short disp = (unsigned short)(fx * base * 64 / img_depth.at<float>(y, x));
                if (img_disp.at<unsigned short>(i, j) < disp)
                {
                    img_disp.at<unsigned short>(i, j) = disp;
                }
                else
                {
                    continue;
                }
            }
        }
    }

    return img_disp;
}