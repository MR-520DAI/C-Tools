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

//float fx = 7.215377e+02;
//float fy = 7.215377e+02;
//float cx = 6.095593e+02;
//float cy = 1.728540e+02;
//float baseline = 54;

float fx = 1.49975506e+03;
float fy = 1.49975506e+03;
float cx = 2.36479708e+02;
float cy = 1.80688532e+02;
float baseline = 175.6;

int main()
{
    /* 点云恢复 */
    PointCloud<PointXYZRGB> cloud_a;
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

    Mat color = imread("./color.jpg");
    Mat disp = imread("./disp.jpg", 0);

    Mat depth;
    depth.create(disp.rows, disp.cols, CV_16UC1);
    for (int i = 0; i < disp.rows; i++)
    {
        for (int j = 0; j < disp.cols; j++)
        {
            if (disp.ptr<unsigned char>(i)[j] != 0)
            {
                depth.ptr<unsigned short>(i)[j] = (unsigned short)(fx * baseline / disp.ptr<unsigned char>(i)[j] / 4);
                //cout << (unsigned short)(fx * baseline / disp.ptr<unsigned char>(i)[j] / 4) << endl;
            }
            else
            {
                depth.ptr<unsigned short>(i)[j] = 0;
            }
        }
    }

    cloud_a.height = color.rows;
    cloud_a.width = color.cols;
    cloud_a.points.resize(color.rows * color.cols);

    /*fstream txt;
    txt.open("./xyz.txt", ios::out);
    txt << "% x" << "\t" << "y" << "\t" << "z\n";*/
    double Xw = 0, Yw = 0, Zw = 0;
    for (int i = 0; i < color.rows; i++)
    {
        for (int j = 0; j < color.cols; j++)
        {
            unsigned int num = i * color.cols + j;
            unsigned short d = depth.ptr<unsigned short>(i)[j]; // 深度值
            //cout << "d:" << d << endl;
            if (d == 0) continue; // 为0表示没有测量到
            if (d > 100000) continue;

            Zw = double(d) / 1000;
            Xw = ((double)i - cx) * Zw / fx;
            Yw = ((double)j - cy) * Zw / fy;
            //txt << Xw << "\t" << Yw << "\t" << Zw << endl;

            cloud_a.points[num].b = color.at<Vec3b>(i, j)[0];
            cloud_a.points[num].g = color.at<Vec3b>(i, j)[1];
            cloud_a.points[num].r = color.at<Vec3b>(i, j)[2];

            cloud_a.points[num].x = Xw;
            cloud_a.points[num].y = Yw;
            cloud_a.points[num].z = Zw;
        }
    }
    //txt.close();

    *cloud = cloud_a;

    visualization::CloudViewer viewer("Cloud Viewer");

    viewer.showCloud(cloud);

    viewer.runOnVisualizationThreadOnce(viewerOneOff);

    while (!viewer.wasStopped())
    {

    }

    return 0;
}