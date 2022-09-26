#include <iostream>
#include <fstream>
#include <vector>
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
	/* 点云数据初始化 */
	double x;
	PointCloud<PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_rgb->points.resize(68);

	ifstream inFile;
	inFile.open("3Ddata_T.txt");
	int loc = 0;
	int num = 0;
	while (inFile >> x)
	{
		if (loc != 0 && loc % 3 == 0)
		{
			num++;
		}
		switch (loc % 3)
		{
		case 0:
			cloud_rgb->points[num].x = x * 10;
			loc++;
			break;
		case 1:
			cloud_rgb->points[num].y = x * 10;
			loc++;
			break;
		case 2:
			cloud_rgb->points[num].z = x * 10;
			loc++;
			break;
		default:
			break;
		}
	}

	for (int i = 0; i < cloud_rgb->size(); i++)
	{
		cloud_rgb->points[i].b = 0;
		cloud_rgb->points[i].g = 255;
		cloud_rgb->points[i].r = 0;
	}

	visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(cloud_rgb);
	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	while (!viewer.wasStopped())
	{

	}

	return 0;
}
