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

void viewerOneOff(visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
}

int main()
{
    /* 相机参数 */
    // K代表相机内参矩阵
    Mat K = (Mat_<double>(3, 4) << 3.5204932328875097e+03, 0., 6.7804628181009662e+02, 0,
        0, 2.9619494807634046e+03, 3.6969187268740455e+02, 0,
        0, 0, 1, 0);
    // T代表激光雷达与相机的变换矩阵（相机外参）
    Mat T = (Mat_<double>(4, 4) << -0.0365625, - 0.999266, 0.0114496, 0.100694,
        -0.000640202, -0.0114338, -0.999934, 0.0368182,
        0.999331, -0.0365674, -0.000221682, 0.0229261,
        0, 0, 0, 1);

    /* 点云数据读取 */
    PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloud<PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("15.pcd", *cloud) == -1)//*打开点云文件
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return(-1);
    }

    cloud_rgb->points.resize(cloud->size());
    cout << cloud->size() << endl;

    /* 图像数据读取 */
    Mat img_rgb = imread("15.jpg");

    for (int i = 0; i < cloud->size(); i++)
    {
        /* 点云处理 */
        cloud_rgb->points[i].x = cloud->points[i].x;
        cloud_rgb->points[i].y = cloud->points[i].y;
        cloud_rgb->points[i].z = cloud->points[i].z;

        // 激光雷达坐标系（world_point）通过左乘外参矩阵、内参矩阵，转换到像素坐标系下（img_point）
        Mat world_point = (Mat_<double>(4, 1) << (double)cloud->points[i].x, (double)cloud->points[i].y, (double)cloud->points[i].z, 1);
        Mat img_point = K * T * world_point;

        int x = (int)(img_point.at<double>(0, 0) / img_point.at<double>(2, 0));
        int y = (int)(img_point.at<double>(1, 0) / img_point.at<double>(2, 0));

        // 判断是否越界
        if (x >= 0 && x < img_rgb.cols && y >= 0 && y < img_rgb.rows)
        {
            cloud_rgb->points[i].b = img_rgb.at<Vec3b>(y, x)[0];
            cloud_rgb->points[i].g = img_rgb.at<Vec3b>(y, x)[1];
            cloud_rgb->points[i].r = img_rgb.at<Vec3b>(y, x)[2];
        }
        else
        {
            cloud_rgb->points[i].b = 0;
            cloud_rgb->points[i].g = 0;
            cloud_rgb->points[i].r = 0;
        }
    }

    /* 可视化 */
    visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud_rgb);
    viewer.runOnVisualizationThreadOnce(viewerOneOff);

    while (!viewer.wasStopped())
    {

    }

    return 0;
}