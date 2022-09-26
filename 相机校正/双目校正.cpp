#include<fstream>
#include<iostream>
#include<opencv2/opencv.hpp>

using namespace cv;
using namespace std;

void GetMatrix(float *R, float *P, float *PRI);
void GetMap(float *K, float *D, float *PRI, float *mapx, float *mapy, Size size);
void Imgremap(Mat srcImg, Mat &dstImg, float *mapx, float *mapy);
void Imgremap_new(Mat srcImg, Mat &dstImg, float *mapx, float *mapy);
void Imgremap_insert(Mat srcImg, Mat &dstImg, float *mapx, float *mapy, int scale);
void Imgremap_symmetric(Mat srcImg, Mat &dstImg, float *mapx, float *mapy);

int main()
{
	Mat API_imgl = imread("left0.jpg");
	Mat API_imgr = imread("right0.jpg");

	Size imgsz(640, 360);
	/* 相机参数通过单目、双目标定即可获得 */
	Mat cameraMatrixL = (Mat_<double>(3, 3) << 1744.54370311182, 0., 246.428055511667, 0.,
		1466.32076115206, 164.759318864494, 0., 0., 1.);
	Mat distCoeffL = (Mat_<double>(5, 1) << -0.673059881080621, 1.86108101211671,
		0.00423242768959299, 0.0109307147251052,
		-17.3169547606579);

	Mat API_Rl = (Mat_<double>(3, 3) << 0.99944334, -0.00296422,
		-0.03322975, 0.0028535,
		0.99999022, -0.00337887,
		0.03323944, 0.00328217,
		0.99944203);

	Mat API_Pl = (Mat_<double>(3, 4) << 1.78837464e+03, 0., 2.88095383e+02, 0., 0.,
		1.78837464e+03, 1.67255299e+02, 0., 0., 0., 1.,
		0.);

	Mat cameraMatrixR = (Mat_<double>(3, 3) << 1748.10827299740, 0., 258.271301166744, 0.,
		1470.86121783703, 174.834797383628, 0., 0., 1.);
	Mat distCoeffR = (Mat_<double>(5, 1) << -0.748517583309580, 12.2896519709071,
		0.00201543906843687, 0.0106947546831789,
		-339.960584471901);

	Mat API_Rr = (Mat_<double>(3, 3) << 0.99950244, 0.01170687,
		-0.02928863, -0.01160923,
		0.99992648, 0.00350145,
		0.02932747, -0.00315969,
		0.99956486);

	Mat API_Pr = (Mat_<double>(3, 4) << 1.78837464e+03, 0., 2.88095383e+02,
		-2.07342709e+05, 0., 1.78837464e+03,
		1.67255299e+02, 0., 0., 0., 1., 0.);

	Mat API_maplx, API_maply, API_maprx, API_mapry;
	initUndistortRectifyMap(cameraMatrixL, distCoeffL, API_Rl, API_Pl, imgsz, CV_32FC1, API_maplx, API_maply);
	initUndistortRectifyMap(cameraMatrixR, distCoeffR, API_Rr, API_Pr, imgsz, CV_32FC1, API_maprx, API_mapry);
	clock_t start_API, end_API;

	Mat API_unimgl, API_unimgr;
	start_API = clock();
	remap(API_imgl, API_unimgl, API_maplx, API_maply, INTER_LINEAR);
	remap(API_imgr, API_unimgr, API_maprx, API_mapry, INTER_LINEAR);
	end_API = clock();
	cout << "API run time:" << (double)(end_API - start_API) << endl;
	
	imwrite("left_rec.jpg", API_unimgl);
	imwrite("right_rec.jpg", API_unimgr);


	/* 左相机参数 */
	float Kl[3][3] = { { 5.3340331777463712e+02, 0., 3.4251343227755160e+02 },
					   { 0.0, 5.3343402398745684e+02, 2.3475353096292952e+02 },
					   { 0., 0., 1. } };
	float Dl[5] = { -2.8214652038759541e-01, 4.0840748605552028e-02, 1.2058218262263004e-03, -1.2307876204068898e-04,
	   1.1409651538056684e-01 };
	float Rl[3][3] = { { 9.9995545010198061e-01, -7.5529503409555278e-03, 5.6613384011591078e-03 },
					   { 7.5729426565523507e-03, 9.9996513545382970e-01, -3.5182973615490811e-03 },
					   { -5.6345674959085556e-03, 3.5610136128317303e-03, 9.9997778516884228e-01 } };
	float Pl[3][3] = { { 5.3500952177482827e+02, 0., 3.3665814208984375e+02 },
					   { 0.0, 5.3500952177482827e+02, 2.4442177581787109e+02 },
					   { 0., 0., 1. } };
	/* 右相机参数 */
	float Kr[3][3] = { { 5.3699964365956180e+02, 0., 3.2744774682047540e+02 },
					   { 0.0, 5.3658501956219982e+02, 2.4990007115682096e+02 },
					   { 0., 0., 1. } };
	float Dr[5] = { -2.9611763213840986e-01, 1.3891105660442912e-01, -5.0433529470851200e-04, 9.4658617944131683e-05,
	   -4.9061152399050519e-02 };
	float Rr[3][3] = { { 9.9993738772164931e-01, -1.1094841523320539e-02, 1.4577818686240377e-03 },
					   { 1.1089611831016350e-02, 9.9993221439532998e-01, 3.5478336896012114e-03 },
					   { -1.4970457045358340e-03, -3.5314453165933707e-03, 9.9999264384701070e-01 } };
	float Pr[3][3] = { { 5.3500952177482827e+02, 0., 3.3665814208984375e+02 },
					   { 0.0, 5.3500952177482827e+02, 2.4442177581787109e+02 },
					   { 0., 0., 1. } };

	/* 求逆矩阵 */
	float PRIl[3][3];
	float PRIr[3][3];
	memset(PRIl, 0, sizeof(PRIl));
	memset(PRIr, 0, sizeof(PRIr));
	GetMatrix((float*)Rl, (float*)Pl, (float*)PRIl);
	GetMatrix((float*)Rr, (float*)Pr, (float*)PRIr);

	/* 获取映射表 */
	int scale = 4;
	Size ImgSize(640, 480);
	float *maplx = new float[ImgSize.width * ImgSize.height];
	float *maply = new float[ImgSize.width * ImgSize.height];
	float *maprx = new float[ImgSize.width * ImgSize.height];
	float *mapry = new float[ImgSize.width * ImgSize.height];
	float *maplx_scale = new float[(ImgSize.width/scale+1) * (ImgSize.height/scale+1)];
	float *maply_scale = new float[(ImgSize.width / scale + 1) * (ImgSize.height / scale + 1)];
	float *maprx_scale = new float[(ImgSize.width / scale + 1) * (ImgSize.height / scale + 1)];
	float *mapry_scale = new float[(ImgSize.width / scale + 1) * (ImgSize.height / scale + 1)];

	GetMap((float*)Kl, (float*)Dl, (float*)PRIl, maplx, maply, ImgSize);
	GetMap((float*)Kr, (float*)Dr, (float*)PRIr, maprx, mapry, ImgSize);


	/* 输出数据到TXT */
	/*ofstream outfilex("mapx.txt", ios::app);
	ofstream outfiley("mapy.txt", ios::app);
	for (int i = 0; i < ImgSize.height; i++)
	{
		for (int j = 0; j < ImgSize.width; j++)
		{
			outfilex << maplx[i*ImgSize.width + j] << " ";
		}
		outfilex << "\n";
	}
	for (int i = 0; i < ImgSize.height; i++)
	{
		for (int j = 0; j < ImgSize.width; j++)
		{
			outfiley << maply[i*ImgSize.width + j] << " ";
		}
		outfiley << "\n";
	}*/

	/* 生成压缩映射表 */
	for (int i = 0; i < ImgSize.height / scale; i++)
	{
		for (int j = 0; j < ImgSize.width / scale; j++)
		{
			maplx_scale[i * (ImgSize.width / scale + 1) + j] = maplx[(i * scale) * ImgSize.width + j * scale];

			if (j == (ImgSize.width / scale - 1))
			{
				maplx_scale[i * (ImgSize.width / scale + 1) + ImgSize.width / scale] = maplx[(i * scale) * ImgSize.width + ImgSize.width-1];
			}
		}
		if (i == (ImgSize.height / scale - 1))
		{
			for (int j = 0; j < ImgSize.width / scale; j++)
			{
				maplx_scale[(i + 1)*(ImgSize.width / scale + 1) + j] = maplx[(ImgSize.height - 1)*ImgSize.width + j*scale];
				if (j == (ImgSize.width / scale - 1))
				{
					maplx_scale[(i + 1)*(ImgSize.width / scale + 1) + ImgSize.width / scale] = maplx[(ImgSize.height - 1)*ImgSize.width + ImgSize.width - 1];
				}
			}
		}
	}



	for (int i = 0; i < ImgSize.height / scale; i++)
	{
		for (int j = 0; j < ImgSize.width / scale; j++)
		{
			maply_scale[i * (ImgSize.width / scale + 1) + j] = maply[(i * scale) * ImgSize.width + j * scale];

			if (j == (ImgSize.width / scale - 1))
			{
				maply_scale[i * (ImgSize.width / scale + 1) + ImgSize.width / scale] = maply[(i * scale) * ImgSize.width + ImgSize.width - 1];
			}
		}
		if (i == (ImgSize.height / scale - 1))
		{
			for (int j = 0; j < ImgSize.width / scale; j++)
			{
				maply_scale[(i + 1)*(ImgSize.width / scale + 1) + j] = maply[(ImgSize.height - 1)*ImgSize.width + j*scale];
				if (j == (ImgSize.width / scale - 1))
				{
					maply_scale[(i + 1)*(ImgSize.width / scale + 1) + ImgSize.width / scale] = maply[(ImgSize.height - 1)*ImgSize.width + ImgSize.width - 1];
				}
			}
		}
	}
	ofstream outfilex("mapx.txt", ios::app);
	ofstream outfiley("mapy.txt", ios::app);
	for (int i = 0; i < (ImgSize.height / scale + 1); i++)
	{
		for (int j = 0; j < (ImgSize.width / scale + 1); j++)
		{
			outfilex << maplx_scale[i*(ImgSize.width/scale+1) + j] << " ";
		}
		outfilex << "\n";
	}
	outfilex.close();
	for (int i = 0; i < (ImgSize.height / scale + 1); i++)
	{
		for (int j = 0; j < (ImgSize.width / scale + 1); j++)
		{
			outfiley << maply_scale[i*(ImgSize.width/scale+1) + j] << " ";
		}
		outfiley << "\n";
	}
	outfiley.close();
	/* 映射校正 */
	clock_t start_MY, end_MY;

	//Mat imgl = imread("left_sample.jpg", 0);
	//Mat imgr = imread("right_sample.jpg", 0);
	//Mat unimgl(imgl.rows, imgl.cols, CV_8UC1);
	//Mat unimgr(imgl.rows, imgl.cols, CV_8UC1);

	/*start_MY = clock();
	Imgremap(imgl, unimgl, maplx, maply);
	Imgremap(imgr, unimgr, maprx, mapry);
	end_MY = clock();
	cout << "MY run time:" << (double)(end_MY - start_MY) << endl;

	imwrite("left01_MY.jpg", unimgl);
	imwrite("right01_MY.jpg", unimgr);*/

	/* 压缩映射表校正 */
	Mat imgl = imread("left_sample.jpg", 0);
	Mat imgr = imread("right01.jpg", 0);
	Mat unimgl(imgl.rows, imgl.cols, CV_8UC1);
	Mat unimgr(imgl.rows, imgl.cols, CV_8UC1);

	Imgremap_insert(imgl, unimgl, maplx_scale, maply_scale, scale);
	imwrite("left01_MY_yasuo_8.jpg", unimgl);


	delete[] maplx;
	delete[] maply;
	delete[] maprx;
	delete[] mapry;
	delete[] maplx_scale;
	delete[] maply_scale;
	delete[] maprx_scale;
	delete[] mapry_scale;

	system("pause");
	return 0;
}

/*
* brief 获取矩阵PR的逆矩阵
* param R	输入，旋转矩阵指针，双目标定获得
* param P	输入，映射矩阵指针，双目标定获得
* param PRI 输出，逆矩阵指针

*/
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

/*
* brief 获取映射表
* param K		输入，相机内参矩阵指针，单目标定获得
* param D		输入，相机畸变矩阵指针，单目标定获得
* param PRI		输入，P*R的逆矩阵
* param mapx	输出，x坐标映射表
* param mapy	输出，y坐标映射表
* param size	输入，图像分辨率

*/
void GetMap(float *K, float *D, float *PRI, float *mapx, float *mapy, Size size)
{
	float fx = K[0];
	float fy = K[4];
	float u0 = K[2];
	float v0 = K[5];
	float k1 = D[0];
	float k2 = D[1];
	float p1 = D[2];
	float p2 = D[3];
	float k3 = D[4];

	/* 学术大佬研究的模型：去除畸变+双目校正 */
	for (int i = 0; i < size.height; i++)
	{
		float _x = i*PRI[1] + PRI[2];
		float _y = i*PRI[4] + PRI[5];
		float _w = i*PRI[7] + PRI[8];

		for (int j = 0; j < size.width; j++, _x += PRI[0] , _y += PRI[3], _w += PRI[6])
		{
			float w = 1. / _w;
			float x = _x * w;
			float y = _y * w;
			float x2 = x * x;
			float y2 = y * y;
			float r2 = x2 + y2;
			float _2xy = 2 * x * y;
			float kr = 1 + ((k3*r2 + k2)*r2 + k1)*r2;
			float u = fx*(x*kr + p1*_2xy + p2*(r2 + 2 * x2)) + u0;
			float v = fy*(y*kr + p1*(r2 + 2 * y2) + p2*_2xy) + v0;

			u = floor(u * 16) / 16;
			v = floor(v * 16) / 16;

			mapx[i*size.width + j] = u;
			mapy[i*size.width + j] = v;

		}
	}
}

/*
* brief 执行校正过程
* param srcImg	输入，原图数据
* param dstImg	输入，校正后图像数据
* param mapx	输入，x坐标映射表
* param mapy	输入，y坐标映射表

*/
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

			if (x > 1 && x < (srcImg.cols-1) && y > 1 && y < (srcImg.rows-1))
			{
				u = mapx[i*srcImg.cols + j] - x;
				v = mapy[i*srcImg.cols + j] - y;
				dstImg.ptr<uchar>(i)[j] = (uchar)((1 - u)*(1 - v)*srcImg.ptr<uchar>(int(y))[int(x)] + (1 - u)*v*srcImg.ptr<uchar>(int(y + 1))[int(x)]
					+ u*(1 - v)*srcImg.ptr<uchar>(int(y))[int(x + 1)] + u*v*srcImg.ptr<uchar>(int(y + 1))[int(x + 1)]);
				//cout << (int)(dstImg.ptr<uchar>(i)[j]) << endl;
			}
			else
			{
				dstImg.ptr<uchar>(i)[j] = 0;
			}

		}
	}
}


/*
* brief 执行校正过程
* param srcImg	输入，原图数据
* param dstImg	输入，校正后图像数据
* param mapx	输入，x坐标映射表
* param mapy	输入，y坐标映射表

*/
void Imgremap_new(Mat srcImg, Mat &dstImg, float *mapx, float *mapy)
{
	/* 浮点转定点运算，执行映射+插值过程 */
	for (int i = 0; i < srcImg.rows; ++i)
	{
		for (int j = 0; j < srcImg.cols; ++j)
		{
			int pdata = i*srcImg.cols + j;
			int x = (int)mapx[pdata];
			int y = (int)mapy[pdata];
			short PartX = (mapx[pdata] - x) * 2048;
			short PartY = (mapy[pdata] - y) * 2048;
			short InvX = 2048 - PartX;
			short InvY = 2048 - PartY;

			if (x > 1 && x < (srcImg.cols - 1) && y > 1 && y < (srcImg.rows - 1))
			{
				dstImg.ptr<uchar>(i)[j] = (((InvX*srcImg.ptr<uchar>(y)[x] + PartX*srcImg.ptr<uchar>(y)[x+1])*InvY +
					(InvX*srcImg.ptr<uchar>(y+1)[x] + PartX*srcImg.ptr<uchar>(y + 1)[x + 1])*PartY) >> 22);
				//cout << (int)(dstImg.ptr<uchar>(i)[j]) << endl;
			}
			else
			{
				dstImg.ptr<uchar>(i)[j] = 0;
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
			/*for (int m = 0; m < srcImg.cols / scale; m++)
			{
				cout << *(datax_i + m) << " ";
			}
			cout << endl;*/
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
				dstImg.ptr<uchar>(i)[j] = (uchar)((1 - u)*(1 - v)*srcImg.ptr<uchar>(int(y))[int(x)] + (1 - u)*v*srcImg.ptr<uchar>(int(y + 1))[int(x)]
					+ u*(1 - v)*srcImg.ptr<uchar>(int(y))[int(x + 1)] + u*v*srcImg.ptr<uchar>(int(y + 1))[int(x + 1)]);
				//cout << (int)(dstImg.ptr<uchar>(i)[j]) << endl;
			}
			else
			{
				dstImg.ptr<uchar>(i)[j] = 0;
			}

		}

	}
}

void Imgremap_symmetric(Mat srcImg, Mat &dstImg, float *mapx, float *mapy)
{
	float x1, y1, x2, y2, x3, y3, x4, y4;
	float u1, v1, u2, v2, u3, v3, u4, v4;

	/* 纯浮点运算，执行映射+插值过程 */
	for (int i = 0; i < srcImg.rows/2; i++)
	{
		for (int j = 0; j < srcImg.cols/2; j++)
		{
			x1 = mapx[i*srcImg.cols + j];
			y1 = mapy[i*srcImg.cols + j];
			x2 = srcImg.cols - x1;
			y2 = y1;
			x3 = x2;
			y3 = srcImg.rows - y2;
			x4 = x1;
			y4 = srcImg.rows - y1;

			if (x1 > 1 && x1 < (srcImg.cols - 1) && y1 > 1 && y1 < (srcImg.rows - 1))
			{
				u1 = x1 - (int)x1;
				v1 = y1 - (int)y1;
				dstImg.ptr<uchar>(i)[j] = (uchar)((1 - u1)*(1 - v1)*srcImg.ptr<uchar>(int(y1))[int(x1)] + (1 - u1)*v1*srcImg.ptr<uchar>(int(y1 + 1))[int(x1)]
					+ u1*(1 - v1)*srcImg.ptr<uchar>(int(y1))[int(x1 + 1)] + u1*v1*srcImg.ptr<uchar>(int(y1 + 1))[int(x1 + 1)]);
			}
			else
			{
				dstImg.ptr<uchar>(i)[j] = 0;
			}

			if (x2 > 1 && x2 < (srcImg.cols - 1) && y2 > 1 && y2 < (srcImg.rows - 1))
			{
				u2 = x2 - (int)x2;
				v2 = y2 - (int)y2;
				dstImg.ptr<uchar>(i)[639-j] = (uchar)((1 - u2)*(1 - v2)*srcImg.ptr<uchar>(int(y2))[int(x2)] + (1 - u2)*v2*srcImg.ptr<uchar>(int(y2 + 1))[int(x2)]
					+ u2*(1 - v2)*srcImg.ptr<uchar>(int(y2))[int(x2 + 1)] + u2*v2*srcImg.ptr<uchar>(int(y2 + 1))[int(x2 + 1)]);
			}
			else
			{
				dstImg.ptr<uchar>(i)[639-j] = 0;
			}

			if (x3 > 1 && x3 < (srcImg.cols - 1) && y3 > 1 && y3 < (srcImg.rows - 1))
			{
				u3 = x3 - (int)x3;
				v3 = y3 - (int)y3;
				dstImg.ptr<uchar>(479-i)[639-j] = (uchar)((1 - u3)*(1 - v3)*srcImg.ptr<uchar>(int(y3))[int(x3)] + (1 - u3)*v3*srcImg.ptr<uchar>(int(y3 + 1))[int(x3)]
					+ u3*(1 - v3)*srcImg.ptr<uchar>(int(y3))[int(x3 + 1)] + u3*v3*srcImg.ptr<uchar>(int(y3 + 1))[int(x3 + 1)]);
			}
			else
			{
				dstImg.ptr<uchar>(479-i)[639-j] = 0;
			}

			if (x4 > 1 && x4 < (srcImg.cols - 1) && y4 > 1 && y4 < (srcImg.rows - 1))
			{
				u4 = x4 - (int)x4;
				v4 = y4 - (int)y4;
				dstImg.ptr<uchar>(479-i)[j] = (uchar)((1 - u4)*(1 - v4)*srcImg.ptr<uchar>(int(y4))[int(x4)] + (1 - u4)*v4*srcImg.ptr<uchar>(int(y4 + 1))[int(x4)]
					+ u4*(1 - v4)*srcImg.ptr<uchar>(int(y4))[int(x4 + 1)] + u4*v4*srcImg.ptr<uchar>(int(y4 + 1))[int(x4 + 1)]);
			}
			else
			{
				dstImg.ptr<uchar>(479-i)[j] = 0;
			}
		}
	}
}

//void InitUndistortRectifyMap(InputArray _cameraMatrix, InputArray _distCoeffs,
//	InputArray _matR, InputArray _newCameraMatrix,
//	Size size, int m1type, OutputArray _map1, OutputArray _map2)
//{
//	//相机内参、畸变矩阵
//	Mat cameraMatrix = _cameraMatrix.getMat(), distCoeffs = _distCoeffs.getMat();
//	cout << "cameraMatrix" << cameraMatrix << endl;
//	cout << "distCoeffs" << distCoeffs << endl;
//	//旋转矩阵、投影矩阵
//	Mat matR = _matR.getMat(), newCameraMatrix = _newCameraMatrix.getMat();
//	cout << "matR" << matR << endl;
//	cout << "newCameraMatrix" << newCameraMatrix << endl;
//	if (m1type <= 0)
//		m1type = CV_16SC2;
//	CV_Assert(m1type == CV_16SC2 || m1type == CV_32FC1 || m1type == CV_32FC2);
//	_map1.create(size, m1type);
//	Mat map1 = _map1.getMat(), map2;
//	if (m1type != CV_32FC2)
//	{
//		_map2.create(size, m1type == CV_16SC2 ? CV_16UC1 : CV_32FC1);
//		map2 = _map2.getMat();
//	}
//	else
//		_map2.release();
//
//	Mat_<double> R = Mat_<double>::eye(3, 3);
//	//A为相机内参
//	Mat_<double> A = Mat_<double>(cameraMatrix), Ar;
//
//	//Ar 为摄像机坐标参数
//	if (newCameraMatrix.data)
//		Ar = Mat_<double>(newCameraMatrix);
//	else
//		Ar = getDefaultNewCameraMatrix(A, size, true);
//	cout << "Ar" << Ar << endl;
//	//R  为旋转矩阵
//	if (matR.data)
//		R = Mat_<double>(matR);
//	cout << "R" << R << endl;
//	//distCoeffs为畸变矩阵
//	if (distCoeffs.data)
//		distCoeffs = Mat_<double>(distCoeffs);
//	else
//	{
//		distCoeffs.create(8, 1, CV_64F);
//		distCoeffs = 0.;
//	}
//	cout << "distCoeffs" << distCoeffs << endl;
//
//	CV_Assert(A.size() == Size(3, 3) && A.size() == R.size());
//	CV_Assert(Ar.size() == Size(3, 3) || Ar.size() == Size(4, 3));
//
//	//摄像机坐标系第四列参数  旋转向量转为旋转矩阵
//	Mat_<double> iR = (Ar.colRange(0, 3)*R).inv(DECOMP_LU);
//	cout << "iR:" << iR << endl;
//	//ir IR矩阵的指针
//	const double* ir = &iR(0, 0);
//	//获取相机的内参 u0  v0 为主坐标点   fx fy 为焦距
//	double u0 = A(0, 2), v0 = A(1, 2);
//	double fx = A(0, 0), fy = A(1, 1);
//
//	CV_Assert(distCoeffs.size() == Size(1, 4) || distCoeffs.size() == Size(4, 1) ||
//		distCoeffs.size() == Size(1, 5) || distCoeffs.size() == Size(5, 1) ||
//		distCoeffs.size() == Size(1, 8) || distCoeffs.size() == Size(8, 1));
//
//	if (distCoeffs.rows != 1 && !distCoeffs.isContinuous())
//		distCoeffs = distCoeffs.t();
//
//	//畸变参数计算
//	double k1 = ((double*)distCoeffs.data)[0];
//	double k2 = ((double*)distCoeffs.data)[1];
//	double p1 = ((double*)distCoeffs.data)[2];
//	double p2 = ((double*)distCoeffs.data)[3];
//	double k3 = distCoeffs.cols + distCoeffs.rows - 1 >= 5 ? ((double*)distCoeffs.data)[4] : 0.;
//	double k4 = distCoeffs.cols + distCoeffs.rows - 1 >= 8 ? ((double*)distCoeffs.data)[5] : 0.;
//	double k5 = distCoeffs.cols + distCoeffs.rows - 1 >= 8 ? ((double*)distCoeffs.data)[6] : 0.;
//	double k6 = distCoeffs.cols + distCoeffs.rows - 1 >= 8 ? ((double*)distCoeffs.data)[7] : 0.;
//	//图像高度
//	for (int i = 0; i < size.height; i++)
//	{
//		//映射矩阵map1 
//		float* m1f = (float*)(map1.data + map1.step*i);
//		//映射矩阵map2
//		float* m2f = (float*)(map2.data + map2.step*i);
//		short* m1 = (short*)m1f;
//		ushort* m2 = (ushort*)m2f;
//		//摄像机参数矩阵最后一列向量转换成的3*3矩阵参数
//		double _x = i*ir[1] + ir[2];
//		double _y = i*ir[4] + ir[5];
//		double _w = i*ir[7] + ir[8];
//
//		//图像宽度
//		for (int j = 0; j < size.width; j++, _x += ir[0], _y += ir[3], _w += ir[6])
//		{
//			//获取摄像机坐标系第四列参数
//			double w = 1. / _w, x = _x*w, y = _y*w;
//			double x2 = x*x, y2 = y*y;
//			double r2 = x2 + y2, _2xy = 2 * x*y;
//			double kr = (1 + ((k3*r2 + k2)*r2 + k1)*r2) / (1 + ((k6*r2 + k5)*r2 + k4)*r2);
//			double u = fx*(x*kr + p1*_2xy + p2*(r2 + 2 * x2)) + u0;
//			double v = fy*(y*kr + p1*(r2 + 2 * y2) + p2*_2xy) + v0;
//			cout << "u:" << u << "v:" << v << endl;
//			if (m1type == CV_16SC2)
//			{
//				int iu = saturate_cast<int>(u*INTER_TAB_SIZE);
//				int iv = saturate_cast<int>(v*INTER_TAB_SIZE);
//				m1[j * 2] = (short)(iu >> INTER_BITS);
//				m1[j * 2 + 1] = (short)(iv >> INTER_BITS);
//				m2[j] = (ushort)((iv & (INTER_TAB_SIZE - 1))*INTER_TAB_SIZE + (iu & (INTER_TAB_SIZE - 1)));
//			}
//			else if (m1type == CV_32FC1)
//			{
//				m1f[j] = (float)u;
//				m2f[j] = (float)v;
//			}
//			else
//			{
//				m1f[j * 2] = (float)u;
//				m1f[j * 2 + 1] = (float)v;
//			}
//		}
//	}
//}
