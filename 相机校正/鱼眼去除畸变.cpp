#include<fstream>
#include<iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void GetMatrix(float *R, float *P, float *PRI);
void GetMap(float *K, float *D, float *PRI, float *mapx, float *mapy, Size size);
void Imgremap(Mat srcImg, Mat &dstImg, float *mapx, float *mapy);
void Imgremap_insert(Mat srcImg, Mat &dstImg, float *mapx, float *mapy, int scale);
void GetMap_height(float* mapx, float* mapy, Size size);

int main()
{
	Mat Img = imread("0.bmp");
	Size image_size = Size(1280, 960);

	/* opencv API方式 */
	/*Mat cameraMatrix = (Mat_<double>(3, 3) << 374.3638256307671, 0, 608.3464790531788, 0.,
		374.3626453221136, 469.5921008862477, 0., 0., 1.);
	Mat distCoeffL = (Mat_<double>(4, 1) << -0.114007, 0.0736486, -0.115118, 0.0579822);

	Mat intrinsic_mat(cameraMatrix), cameraMatrix_new;
	intrinsic_mat.copyTo(cameraMatrix_new);
	cameraMatrix_new.at<double>(0, 0) *= 1;
	cameraMatrix_new.at<double>(1, 1) *= 1;
	cameraMatrix_new.at<double>(0, 2) += 0;
	cameraMatrix_new.at<double>(1, 2) += 0;

	Mat UnImg_API;
	fisheye::undistortImage(Img, UnImg_API, cameraMatrix, distCoeffL, cameraMatrix_new, image_size);
	imwrite("0_un_API.jpg", UnImg_API);*/

	/* 自建代码 */
	float K[3][3] = { { 374.3638256307671, 0, 608.3464790531788 },
					   { 0.,374.3626453221136, 469.5921008862477 },
					   { 0., 0., 1. } };
	float D[4] = { -0.114007, 0.0736486, -0.115118, 0.057982 };
	float R[3][3] = { {1, 0, 0},
					  {0, 1, 0},
					  {0, 0, 1} };
	float P[3][3] = { { 374.3638256307671, 0, 608.3464790531788 },
					  { 0.,374.3626453221136, 469.5921008862477 },
					  { 0., 0., 1. } };
	/* 求逆矩阵 */
	float PRI[3][3];
	GetMatrix((float*)R, (float*)P, (float*)PRI);

	/* 获取映射表 */
	int scale = 16;
	float *mapx = new float[image_size.width * image_size.height];
	float *mapy = new float[image_size.width * image_size.height];
	float *mapx_scale = new float[(image_size.width / scale + 1) * (image_size.height / scale + 1)];
	float *mapy_scale = new float[(image_size.width / scale + 1) * (image_size.height / scale + 1)];
	//GetMap((float*)K, (float*)D, (float*)PRI, mapx, mapy, image_size);
	GetMap_height((float*)mapx, (float*)mapy, image_size);

	/* 输出数据到TXT */
	/*ofstream outfilex("mapx.txt", ios::app);
	ofstream outfiley("mapy.txt", ios::app);
	for (int i = 0; i < image_size.height; i++)
	{
		for (int j = 0; j < image_size.width; j++)
		{
			outfilex << mapx[i*image_size.width + j] << " ";
		}
		outfilex << "\n";
	}
	for (int i = 0; i < image_size.height; i++)
	{
		for (int j = 0; j < image_size.width; j++)
		{
			outfiley << mapy[i*image_size.width + j] << " ";
		}
		outfiley << "\n";
	}*/

	/*for (int i = 0; i < image_size.height / scale; i++)
	{
		for (int j = 0; j < image_size.width / scale; j++)
		{
			mapx_scale[i * (image_size.width / scale + 1) + j] = mapx[(i * scale) * image_size.width + j * scale];

			if (j == (image_size.width / scale - 1))
			{
				mapx_scale[i * (image_size.width / scale + 1) + image_size.width / scale] = mapx[(i * scale) * image_size.width + image_size.width - 1];
			}
		}
		if (i == (image_size.height / scale - 1))
		{
			for (int j = 0; j < image_size.width / scale; j++)
			{
				mapx_scale[(i + 1)*(image_size.width / scale + 1) + j] = mapx[(image_size.height - 1)*image_size.width + j*scale];
				if (j == (image_size.width / scale - 1))
				{
					mapx_scale[(i + 1)*(image_size.width / scale + 1) + image_size.width / scale] = mapx[(image_size.height - 1)*image_size.width + image_size.width - 1];
				}
			}
		}
	}

	for (int i = 0; i < image_size.height / scale; i++)
	{
		for (int j = 0; j < image_size.width / scale; j++)
		{
			mapy_scale[i * (image_size.width / scale + 1) + j] = mapy[(i * scale) * image_size.width + j * scale];

			if (j == (image_size.width / scale - 1))
			{
				mapy_scale[i * (image_size.width / scale + 1) + image_size.width / scale] = mapy[(i * scale) * image_size.width + image_size.width - 1];
			}
		}
		if (i == (image_size.height / scale - 1))
		{
			for (int j = 0; j < image_size.width / scale; j++)
			{
				mapy_scale[(i + 1)*(image_size.width / scale + 1) + j] = mapy[(image_size.height - 1)*image_size.width + j*scale];
				if (j == (image_size.width / scale - 1))
				{
					mapy_scale[(i + 1)*(image_size.width / scale + 1) + image_size.width / scale] = mapy[(image_size.height - 1)*image_size.width + image_size.width - 1];
				}
			}
		}
	}*/

	/* 未压缩映射校正 */
	Mat unimg(Img.rows, Img.cols, CV_8UC3);
	Imgremap(Img, unimg, mapx, mapy);

	imwrite("0_un_MY.jpg", unimg);

	/* 压缩后映射校正 */
	/*Mat unimg(Img.rows, Img.cols, CV_8UC3);
	Imgremap_insert(Img, unimg, mapx_scale, mapy_scale, scale);

	imwrite("0_un_MY_yasuo16.jpg", unimg);*/

	delete[] mapx;
	delete[] mapy;
	delete[] mapx_scale;
	delete[] mapy_scale;
	mapx = nullptr;
	mapy = nullptr;
	mapx_scale = nullptr;
	mapy_scale = nullptr;

	system("pause");
	return 0;
}

void GetMatrix(float *R, float *P, float *PRI)
{
	float temp;
	float PR[3][3];
	memset(PR, 0, sizeof(PR));

	/* 矩阵P*R */
	for (int i = 0; i < 3; i++)
	{
		for (int k = 0; k < 3; k++)
		{
			temp = P[i * 3 + k];
			for (int j = 0; j < 3; j++)
			{
				PR[i][j] += temp * R[k * 3 + j];
			}
		}
	}

	/* 3X3矩阵求逆 */
	temp = PR[0][0] * PR[1][1] * PR[2][2] + PR[0][1] * PR[1][2] * PR[2][0] + PR[0][2] * PR[1][0] * PR[2][1] -
		PR[0][2] * PR[1][1] * PR[2][0] - PR[0][1] * PR[1][0] * PR[2][2] - PR[0][0] * PR[1][2] * PR[2][1];
	PRI[0] = (PR[1][1] * PR[2][2] - PR[1][2] * PR[2][1]) / temp;
	PRI[1] = -(PR[0][1] * PR[2][2] - PR[0][2] * PR[2][1]) / temp;
	PRI[2] = (PR[0][1] * PR[1][2] - PR[0][2] * PR[1][1]) / temp;
	PRI[3] = -(PR[1][0] * PR[2][2] - PR[1][2] * PR[2][0]) / temp;
	PRI[4] = (PR[0][0] * PR[2][2] - PR[0][2] * PR[2][0]) / temp;
	PRI[5] = -(PR[0][0] * PR[1][2] - PR[0][2] * PR[1][0]) / temp;
	PRI[6] = (PR[1][0] * PR[2][1] - PR[1][1] * PR[2][0]) / temp;
	PRI[7] = -(PR[0][0] * PR[2][1] - PR[0][1] * PR[2][0]) / temp;
	PRI[8] = (PR[0][0] * PR[1][1] - PR[0][1] * PR[1][0]) / temp;

	return;
}

void GetMap(float *K, float *D, float *PRI, float *mapx, float *mapy, Size size)
{
	float fx = K[0];
	float fy = K[4];
	float u0 = K[2];
	float v0 = K[5];
	float k1 = D[0];
	float k2 = D[1];
	float k3 = D[2];
	float k4 = D[3];

	for (int i = 0; i < size.height; i++)
	{
		float _x = i*PRI[1] + PRI[2];
		float _y = i*PRI[4] + PRI[5];
		float _w = i*PRI[7] + PRI[8];

		for (int j = 0; j < size.width; j++)
		{
			float w = 1. / _w;
			float x = _x * w;
			float y = _y * w;

			float r = sqrt(x*x + y*y);
			float theta = atan(r);

			float theta2 = theta * theta;
			float theta4 = theta2 * theta2;
			float theta6 = theta4 * theta2;
			float theta8 = theta4 * theta4;

			float theta_d = theta * (1 + k1*theta2 + k2*theta4 + k3*theta6 + k4 * theta8);
			float scale = (r == 0) ? 1.0 : theta_d / r;

			float u = fx * x * scale + u0;
			float v = fy * y * scale + v0;
			/*u = floor(u * 16) / 16;
			v = floor(v * 16) / 16;*/
			mapx[i*size.width + j] = u;
			mapy[i*size.width + j] = v;

			_x += PRI[0];
			_y += PRI[3];
			_w += PRI[6];
		}
	}

	return;
}

void GetMap_height(float* mapx, float* mapy, Size size)
{
	int R = size.height / 2;
	int R2 = R * R;
	for (int i = 0; i < size.height; i++)
	{
		for (int j = 0; j < size.width; j++)
		{
			mapx[i * size.width + j] = j;
			mapy[i * size.width + j] = (i - R) / R * sqrt(R2 - (j - R) * (j - R)) + R;
			cout << "列数:" << j << endl;
			cout << (i - R) / R * sqrt(R2 - (j - R) * (j - R)) + R << endl;
		}
	}
}

void Imgremap(Mat srcImg, Mat &dstImg, float *mapx, float *mapy)
{
	int x, y;
	float u, v;

	/* 纯浮点运算，执行映射+插值过程 */
	for (int i = 0; i < srcImg.rows; i++)
	{
		for (int j = 0; j < srcImg.cols; j++)
		{
			x = (int)mapx[i*srcImg.cols + j];
			y = (int)mapy[i*srcImg.cols + j];
			
			if (x > 1 && x < (srcImg.cols - 1) && y > 1 && y < (srcImg.rows - 1))
			{
				u = mapx[i*srcImg.cols + j] - x;
				v = mapy[i*srcImg.cols + j] - y;
				dstImg.at<Vec3b>(i, j)[0] = (uchar)((1 - u)*(1 - v)*srcImg.at<Vec3b>(y, x)[0] + (1 - u)*v*srcImg.at<Vec3b>(y + 1, x)[0] +
					u*(1 - v)*srcImg.at<Vec3b>(y, x + 1)[0] + u*v*srcImg.at<Vec3b>(y + 1, x + 1)[0]);

				dstImg.at<Vec3b>(i, j)[1] = (uchar)((1 - u)*(1 - v)*srcImg.at<Vec3b>(y, x)[1] + (1 - u)*v*srcImg.at<Vec3b>(y + 1, x)[1] +
					u*(1 - v)*srcImg.at<Vec3b>(y, x + 1)[1] + u*v*srcImg.at<Vec3b>(y + 1, x + 1)[1]);

				dstImg.at<Vec3b>(i, j)[2] = (uchar)((1 - u)*(1 - v)*srcImg.at<Vec3b>(y, x)[2] + (1 - u)*v*srcImg.at<Vec3b>(y + 1, x)[2] +
					u*(1 - v)*srcImg.at<Vec3b>(y, x + 1)[2] + u*v*srcImg.at<Vec3b>(y + 1, x + 1)[2]);
				//cout << (int)(dstImg.ptr<uchar>(i)[j]) << endl;
			}
			else
			{
				dstImg.at<Vec3b>(i, j)[0] = 0;
				dstImg.at<Vec3b>(i, j)[1] = 0;
				dstImg.at<Vec3b>(i, j)[2] = 0;
			}

		}
	}
}

void Imgremap_insert(Mat srcImg, Mat &dstImg, float *mapx, float *mapy, int scale)
{
	int x, y;
	float u, v;
	float addx, addy;
	float addx_i, addy_i;

	float *datax = mapx;
	float *datay = mapy;

	float *datax_i = mapx;
	float *datay_i = mapy;

	/* 纯浮点运算，执行映射+插值过程 */
	for (int i = 0; i < srcImg.rows; i++)
	{
		if (i != 0 && i % scale == 0)
		{
			datax_i = datax_i + (srcImg.cols / scale + 1);
			datay_i = datay_i + (srcImg.cols / scale + 1);
			datax = datax_i;
			datay = datay_i;
		}

		else
		{
			if (i != 0)
			{
				for (int k = 0; k < srcImg.cols / scale + 1; k++)
				{
					addx_i = (*(datax_i + srcImg.cols / scale + 1 + k) - *(datax_i + k)) / (scale - (i % scale) + 1);
					addy_i = (*(datay_i + srcImg.cols / scale + 1 + k) - *(datay_i + k)) / (scale - (i % scale) + 1);
					*(datax_i + k) = *(datax_i + k) + addx_i;
					*(datay_i + k) = *(datay_i + k) + addy_i;
				}
				datax = datax_i;
				datay = datay_i;
			}
		}

		for (int j = 0; j < srcImg.cols; j++)
		{
			if (j % scale == 0)
			{
				if (j != 0)
				{
					datax++;
					datay++;
				}
				addx = (*(datax + 1) - *(datax)) / scale;
				addy = (*(datay + 1) - *(datay)) / scale;
			}

			if (j % scale != 0)
			{
				x = (int)(*(datax)+addx * (j % scale));
				y = (int)(*(datay)+addy * (j % scale));
			}
			else
			{
				x = (int)(*(datax));
				y = (int)(*(datay));
			}

			if (x > 1 && x < (srcImg.cols - 1) && y > 1 && y < (srcImg.rows - 1))
			{
				u = (*(datax)+addx * (j % scale)) - x;
				v = (*(datay)+addy * (j % scale)) - y;
				dstImg.at<Vec3b>(i, j)[0] = (uchar)((1 - u)*(1 - v)*srcImg.at<Vec3b>(y, x)[0] + (1 - u)*v*srcImg.at<Vec3b>(y + 1, x)[0] +
					u*(1 - v)*srcImg.at<Vec3b>(y, x + 1)[0] + u*v*srcImg.at<Vec3b>(y + 1, x + 1)[0]);

				dstImg.at<Vec3b>(i, j)[1] = (uchar)((1 - u)*(1 - v)*srcImg.at<Vec3b>(y, x)[1] + (1 - u)*v*srcImg.at<Vec3b>(y + 1, x)[1] +
					u*(1 - v)*srcImg.at<Vec3b>(y, x + 1)[1] + u*v*srcImg.at<Vec3b>(y + 1, x + 1)[1]);

				dstImg.at<Vec3b>(i, j)[2] = (uchar)((1 - u)*(1 - v)*srcImg.at<Vec3b>(y, x)[2] + (1 - u)*v*srcImg.at<Vec3b>(y + 1, x)[2] +
					u*(1 - v)*srcImg.at<Vec3b>(y, x + 1)[2] + u*v*srcImg.at<Vec3b>(y + 1, x + 1)[2]);

			}
			else
			{
				dstImg.at<Vec3b>(i, j)[0] = 0;
				dstImg.at<Vec3b>(i, j)[1] = 0;
				dstImg.at<Vec3b>(i, j)[2] = 0;
			}

		}

	}
}