#include<math.h>
#include<stdio.h>
#include<stdlib.h>

#define Pi					   (3.141592)
#define XM_IA_SUCCESS          (0)

typedef struct
{
	/* 相机外参 */
	float fDistanceGround;  /* 相机与地面的距离(单位mm) */
	float fDistanceLeft;    /* 相机与左侧车轮的距离(单位mm) */
	float fDistanceRight;   /* 相机与右侧车轮的距离(单位mm) */
	float fDistanceFront;   /* 相机与车辆前保险杠的距离(单位mm) */
	float fPitch;           /* 相机俯仰角(弧度制) */
	float fYaw;				/* 相机偏航角(弧度制) */
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

typedef struct
{
	float fDisX;
	float fDisY;
}Distance;

void XM_ADAS_CalCamMatI(float pfCamMat[3][3], float pfCamMatI[3][3]);
void UnDistortion(Point stDisPoint, Point *pstUnDisPoint, XM_ADAS_CAM_CAL_S stCamCal, float threshold);
void XM_ADAS_CalDistance(Point stUnDisPoint, float afCamMatI[3][3], float afCamPitchMatI[3][3], float afCamYawMatI[3][3], float height, float* distance);

int main()
{
	XM_ADAS_CAM_CAL_S stCamCal, *pstCamCal = &stCamCal;
	memset(pstCamCal, 0, sizeof(XM_ADAS_CAM_CAL_S));

	//初始化相机参数，由标定软件提供
	stCamCal.fDistanceGround = 933;
	stCamCal.fIntrinsicFx = 1892.602513;
	stCamCal.fIntrinsicFy = 1892.602513;
	stCamCal.fIntrinsicCx = 1437.863993;
	stCamCal.fIntrinsicCy = 822.3684214;
	stCamCal.fDistortionK1 = -0.286739;
	stCamCal.fDistortionK2 = -0.0337054;
	stCamCal.fDistortionK3 = 0.10722879;
	stCamCal.fDistortionP1 = -0.00005977;
	stCamCal.fDistortionP2 = -0.001538837;
	stCamCal.fVanishPointX = 1298;
	stCamCal.fVanishPointY = 878;
	stCamCal.fPitch = atan((pstCamCal->fIntrinsicCy - pstCamCal->fVanishPointY) / pstCamCal->fIntrinsicFy);
	stCamCal.fYaw = atan(((stCamCal.fIntrinsicCx - stCamCal.fVanishPointX) / stCamCal.fIntrinsicFx) * cos(stCamCal.fPitch));

	float afCamMat[3][3];
	float afCamMatI[3][3];
	float afCamYawMat[3][3];
	float afCamYawMatI[3][3];
	float afCamPitchMat[3][3];
	float afCamPitchMatI[3][3];
	float afDistance[2];
	
	//初始化相机内参矩阵
	afCamMat[0][0] = pstCamCal->fIntrinsicFx;
	afCamMat[0][1] = 0;
	afCamMat[0][2] = pstCamCal->fIntrinsicCx;
	afCamMat[1][0] = 0;
	afCamMat[1][1] = pstCamCal->fIntrinsicFy;
	afCamMat[1][2] = pstCamCal->fIntrinsicCy;
	afCamMat[2][0] = 0;
	afCamMat[2][1] = 0;
	afCamMat[2][2] = 1;

	//初始化相机外参矩阵
	afCamPitchMat[0][0] = 1;
	afCamPitchMat[0][1] = 0;
	afCamPitchMat[0][2] = 0;
	afCamPitchMat[1][0] = 0;
	afCamPitchMat[1][1] = cos(stCamCal.fPitch);
	afCamPitchMat[1][2] = -sin(stCamCal.fPitch);
	afCamPitchMat[2][0] = 0;
	afCamPitchMat[2][1] = sin(stCamCal.fPitch);
	afCamPitchMat[2][2] = cos(stCamCal.fPitch);

	afCamYawMat[0][0] = cos(stCamCal.fYaw);
	afCamYawMat[0][1] = -sin(stCamCal.fYaw);
	afCamYawMat[0][2] = 0;
	afCamYawMat[1][0] = sin(stCamCal.fYaw);
	afCamYawMat[1][1] = cos(stCamCal.fYaw);
	afCamYawMat[1][2] = 0;
	afCamYawMat[2][0] = 0;
	afCamYawMat[2][1] = 0;
	afCamYawMat[2][2] = 1;

	Point stDisPoint, *pstDisPoint = &stDisPoint;		//需测距的目标点(去除畸变前)
	Point stUnDisPoint, *pstUnDisPoint = &stUnDisPoint;	//去除畸变后坐标值

	stDisPoint.iX = 1444;	//目标点初始化
	stDisPoint.iY = 965;
	UnDistortion(stDisPoint, pstUnDisPoint, stCamCal, 0.0001);	//目标点去除畸变

	XM_ADAS_CalCamMatI(afCamMat, afCamMatI);
	XM_ADAS_CalCamMatI(afCamPitchMat, afCamPitchMatI);
	XM_ADAS_CalCamMatI(afCamYawMat, afCamYawMatI);

	XM_ADAS_CalDistance(stUnDisPoint, afCamMatI, afCamPitchMatI, afCamYawMatI, pstCamCal->fDistanceGround, afDistance);
	printf("%f\n", afDistance[0]);
	printf("%f\n", afDistance[1] - 700);

	system("pause");
	return 0;
}

void XM_ADAS_CalCamMatI(float afCamMat[3][3], float afCamMatI[3][3])
{
	//相机内参矩阵行列式
	float fDetCamMat = afCamMat[0][0] * afCamMat[1][1] * afCamMat[2][2] + afCamMat[0][1] * afCamMat[1][2] * afCamMat[2][0] + afCamMat[0][2] * afCamMat[1][0] * afCamMat[2][1] -
		afCamMat[0][2] * afCamMat[1][1] * afCamMat[2][0] - afCamMat[0][1] * afCamMat[1][0] * afCamMat[2][2] - afCamMat[0][0] * afCamMat[1][2] * afCamMat[2][1];
	//通过伴随矩阵求出相机内参矩阵的逆
	afCamMatI[0][0] = (afCamMat[1][1] * afCamMat[2][2] - afCamMat[1][2] * afCamMat[2][1]) / fDetCamMat;
	afCamMatI[1][0] = -(afCamMat[1][0] * afCamMat[2][2] - afCamMat[1][2] * afCamMat[2][0]) / fDetCamMat;
	afCamMatI[2][0] = (afCamMat[1][0] * afCamMat[2][1] - afCamMat[1][1] * afCamMat[2][0]) / fDetCamMat;
	afCamMatI[0][1] = -(afCamMat[0][1] * afCamMat[2][2] - afCamMat[0][2] * afCamMat[2][1]) / fDetCamMat;
	afCamMatI[1][1] = (afCamMat[0][0] * afCamMat[2][2] - afCamMat[0][2] * afCamMat[2][0]) / fDetCamMat;
	afCamMatI[2][1] = -(afCamMat[0][0] * afCamMat[2][1] - afCamMat[0][1] * afCamMat[2][0]) / fDetCamMat;
	afCamMatI[0][2] = (afCamMat[0][1] * afCamMat[1][2] - afCamMat[0][2] * afCamMat[1][1]) / fDetCamMat;
	afCamMatI[1][2] = -(afCamMat[0][0] * afCamMat[1][2] - afCamMat[0][2] * afCamMat[1][0]) / fDetCamMat;
	afCamMatI[2][2] = (afCamMat[0][0] * afCamMat[1][1] - afCamMat[0][1] * afCamMat[1][0]) / fDetCamMat;
}

void XM_ADAS_CalDistance(Point stUnDisPoint, float afCamMatI[3][3], float afCamPitchMatI[3][3], float afCamYawMatI[3][3], float height, float* distance)
{
	float scale;
	//通过相机内参矩阵的逆将像素坐标系转到相机坐标系（此时无尺度信息）
	float afCamPoint[3];
	float afWorldPoint_P[3];
	float afWorldPoint[3];
	afCamPoint[0] = afCamMatI[0][0] * (float)stUnDisPoint.iX + afCamMatI[0][1] * (float)stUnDisPoint.iY + afCamMatI[0][2] * 1;
	afCamPoint[1] = afCamMatI[1][0] * (float)stUnDisPoint.iX + afCamMatI[1][1] * (float)stUnDisPoint.iY + afCamMatI[1][2] * 1;
	afCamPoint[2] = afCamMatI[2][0] * (float)stUnDisPoint.iX + afCamMatI[2][1] * (float)stUnDisPoint.iY + afCamMatI[2][2] * 1;
	printf("afCamPoint[0]:%f \n", afCamPoint[0]);
	printf("afCamPoint[1]:%f \n", afCamPoint[1]);
	printf("afCamPoint[2]:%f \n", afCamPoint[2]);
	//通过相机外参修正相机角度（此时无尺度信息）
	afWorldPoint_P[0] = afCamPitchMatI[0][0] * afCamPoint[0] + afCamPitchMatI[0][1] * afCamPoint[1] + afCamPitchMatI[0][2] * afCamPoint[2];
	afWorldPoint_P[1] = afCamPitchMatI[1][0] * afCamPoint[0] + afCamPitchMatI[1][1] * afCamPoint[1] + afCamPitchMatI[1][2] * afCamPoint[2];
	afWorldPoint_P[2] = afCamPitchMatI[2][0] * afCamPoint[0] + afCamPitchMatI[2][1] * afCamPoint[1] + afCamPitchMatI[2][2] * afCamPoint[2];

	afWorldPoint[0] = afCamYawMatI[0][0] * afWorldPoint_P[0] + afCamYawMatI[0][1] * afWorldPoint_P[1] + afCamYawMatI[0][2] * afWorldPoint_P[2];
	afWorldPoint[1] = afCamYawMatI[1][0] * afWorldPoint_P[0] + afCamYawMatI[1][1] * afWorldPoint_P[1] + afCamYawMatI[1][2] * afWorldPoint_P[2];
	afWorldPoint[2] = afCamYawMatI[2][0] * afWorldPoint_P[0] + afCamYawMatI[2][1] * afWorldPoint_P[1] + afCamYawMatI[2][2] * afWorldPoint_P[2];
	//通过相机与地面的距离获取尺度信息，从而计算该点与相机的深度
	scale = height / afWorldPoint[1];
	distance[0] = afWorldPoint[0] * scale;
	distance[1] = afWorldPoint[2] * scale;
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