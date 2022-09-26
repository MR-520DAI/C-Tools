#include<math.h>
#include<stdio.h>
#include<stdlib.h>

#define Pi 3.1415926

typedef struct
{
	/* 相机外参 */
	float fDistanceGround;  /* 相机与地面的距离(单位mm) */
	float fDistanceLeft;    /* 相机与左侧车轮的距离(单位mm) */
	float fDistanceRight;   /* 相机与右侧车轮的距离(单位mm) */
	float fDistanceFront;   /* 相机与车辆前保险杠的距离(单位mm) */
	float fVanishPointX;	/* 图像消失点X坐标 */
	float fVanishPointY;	/* 图像消失点Y坐标 */

	/* 相机内参 */
	float fIntrinsicFx;     /* 相机内参fx */
	float fIntrinsicFy;     /* 相机内参fy */
	float fIntrinsicCx;     /* 相机内参cx */
	float fIntrinsicCy;     /* 相机内参cy */
	float fDistortionK1;    /* 相机径向畸变参数k1 */
	float fDistortionK2;    /* 相机径向畸变参数k2 */
	float fDistortionK3;    /* 相机径向畸变参数k3 */
	float fDistortionP1;    /* 相机切向畸变参数p1 */
	float fDistortionP2;    /* 相机切向畸变参数p2 */
} XM_ADAS_CAM_CAL_S;

typedef struct
{
	int iX;
	int iY;
}Point;

void UnDistortion(Point stDisPoint, Point *pstUnDisPoint, XM_ADAS_CAM_CAL_S stCamCal, float threshold);
float EstimateDistance(Point stUnDisPoint, XM_ADAS_CAM_CAL_S stCamCal);

int main()
{
	float fdistance = 0;
	XM_ADAS_CAM_CAL_S stCamCal, *pstCamCal = &stCamCal;

	stCamCal.fIntrinsicFx = 1892.602513;	//初始化相机内参
	stCamCal.fIntrinsicCx = 1437.863993;
	stCamCal.fIntrinsicFy = 1892.602513;
	stCamCal.fIntrinsicCy = 822.3684214;
	stCamCal.fDistortionK1 = -0.286739;
	stCamCal.fDistortionK2 = -0.0337054;
	stCamCal.fDistortionP1 = -0.00005977;
	stCamCal.fDistortionP2 = -0.001538837;
	stCamCal.fDistortionK3 = 0.10722879;

	stCamCal.fDistanceGround = 933;			//初始化相机外参
	stCamCal.fVanishPointX = 1340;
	stCamCal.fVanishPointY = 885;

	int Points[12][2];						//目标点坐标初始化，由目标检测模块提供坐标值
	Points[0][0] = 1475;
	Points[0][1] = 1342;
	Points[1][0] = 1444;
	Points[1][1] = 1241;
	Points[2][0] = 1416;
	Points[2][1] = 1187;
	Points[3][0] = 1416;
	Points[3][1] = 1142;
	Points[4][0] = 1408;
	Points[4][1] = 1112;
	Points[5][0] = 1404;
	Points[5][1] = 1087;
	Points[6][0] = 1401;
	Points[6][1] = 1065;
	Points[7][0] = 1391;
	Points[7][1] = 1048;
	Points[8][0] = 1389;
	Points[8][1] = 1035;
	Points[9][0] = 1389;
	Points[9][1] = 1021;
	Points[10][0] = 1396;
	Points[10][1] = 1001;
	Points[11][0] = 1390;
	Points[11][1] = 960;

	Point stDisPoint, *pstDisPoint = &stDisPoint;	//需测距的目标点(去除畸变前)
	Point stUnDisPoint, *pstUnDisPoint = &stUnDisPoint;	//去除畸变后坐标值

	for (int i = 0; i < 12; i++)
	{
		stDisPoint.iX = Points[i][0];
		stDisPoint.iY = Points[i][1];
		UnDistortion(stDisPoint, pstUnDisPoint, stCamCal, 0.0001);
		fdistance = EstimateDistance(stUnDisPoint, stCamCal);
		printf("distance = %f \n", fdistance);
	}
	
	system("pause");
	return 0;
}

/* 不动点迭代法求解非线性方程组，实现去除畸变 */
void UnDistortion(Point stDisPoint, Point *pstUnDisPoint, XM_ADAS_CAM_CAL_S stCamCal, float threshold)
{
	float fTargetX, fTargetY;
	float fthresholdX, fthresholdY;
	float fc1, fc2, fTempX, fTempY, fr;
	fc1 = ((float)(stDisPoint.iX) - stCamCal.fIntrinsicCx) / stCamCal.fIntrinsicFx;
	fc2 = ((float)(stDisPoint.iY) - stCamCal.fIntrinsicCy) / stCamCal.fIntrinsicFy;

	fTempX = ((float)(stDisPoint.iX) - stCamCal.fIntrinsicCx) / stCamCal.fIntrinsicFx;	//不动点初始化
	fTempY = ((float)(stDisPoint.iY) - stCamCal.fIntrinsicCy) / stCamCal.fIntrinsicFy;

	while (1)
	{
		fr = pow(fTempX, 2) + pow(fTempY, 2);
		fTargetX = (fc1 - 2 * stCamCal.fDistortionP1*fTempX*fTempY - stCamCal.fDistortionP2*(3 * pow(fTempX, 2) + pow(fTempY, 2))) / (1 + stCamCal.fDistortionK1*fr + stCamCal.fDistortionK2*pow(fr, 2) + stCamCal.fDistortionK3*pow(fr, 3));
		fTargetY = (fc2 - 2 * stCamCal.fDistortionP2*fTempX*fTempY - stCamCal.fDistortionP1*(3 * pow(fTempY, 2) + pow(fTempX, 2))) / (1 + stCamCal.fDistortionK1*fr + stCamCal.fDistortionK2*pow(fr, 2) + stCamCal.fDistortionK3*pow(fr, 3));
		fr = pow(fTargetX, 2) + pow(fTargetY, 2);
		fthresholdX = (fc1 - 2 * stCamCal.fDistortionP1*fTargetX*fTargetY - stCamCal.fDistortionP2*(3 * pow(fTargetX, 2) + pow(fTargetY, 2))) / (1 + stCamCal.fDistortionK1*fr + stCamCal.fDistortionK2*pow(fr, 2) + stCamCal.fDistortionK3*pow(fr, 3)) - fTargetX;
		fthresholdY = (fc2 - 2 * stCamCal.fDistortionP2*fTargetX*fTargetY - stCamCal.fDistortionP1*(3 * pow(fTargetX, 2) + pow(fTargetY, 2))) / (1 + stCamCal.fDistortionK1*fr + stCamCal.fDistortionK2*pow(fr, 2) + stCamCal.fDistortionK3*pow(fr, 3)) - fTargetY;
		if (fthresholdX < threshold && fthresholdY < threshold)
		{
			break;
		}
		else
		{
			fTempX = fTargetX;
			fTempY = fTargetY;
		}
	}
	pstUnDisPoint->iX = (int)(fTargetX * stCamCal.fIntrinsicFx + stCamCal.fIntrinsicCx);
	pstUnDisPoint->iY = (int)(fTargetY * stCamCal.fIntrinsicFy + stCamCal.fIntrinsicCy);
}

float EstimateDistance(Point stUnDisPoint, XM_ADAS_CAM_CAL_S stCamCal)
{
	float fDistance;
	float fPitch, fyaw, fTargetAngle;

	fPitch = Pi / 2 - atan((stCamCal.fIntrinsicCy - stCamCal.fVanishPointY) / stCamCal.fIntrinsicFy);
	fTargetAngle = atan((stCamCal.fIntrinsicCy - (float)stUnDisPoint.iY) / stCamCal.fIntrinsicFy);
	fyaw = atan((stCamCal.fIntrinsicCx - stCamCal.fVanishPointX) / stCamCal.fIntrinsicFx * cos(atan((stCamCal.fIntrinsicCy - stCamCal.fVanishPointY) / stCamCal.fIntrinsicFy)));
	
	fDistance = stCamCal.fDistanceGround * tan(fPitch + fTargetAngle) * cos(fyaw);

	return fDistance;
}
