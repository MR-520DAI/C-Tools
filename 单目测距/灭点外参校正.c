#include<math.h>
#include<stdio.h>
#include<stdlib.h>

#define Pi 3.1415926

typedef struct
{
	/* ������ */
	float fDistanceGround;  /* ��������ľ���(��λmm) */
	float fDistanceLeft;    /* �������೵�ֵľ���(��λmm) */
	float fDistanceRight;   /* ������Ҳ೵�ֵľ���(��λmm) */
	float fDistanceFront;   /* ����복��ǰ���ոܵľ���(��λmm) */
	float fVanishPointX;	/* ͼ����ʧ��X���� */
	float fVanishPointY;	/* ͼ����ʧ��Y���� */

	/* ����ڲ� */
	float fIntrinsicFx;     /* ����ڲ�fx */
	float fIntrinsicFy;     /* ����ڲ�fy */
	float fIntrinsicCx;     /* ����ڲ�cx */
	float fIntrinsicCy;     /* ����ڲ�cy */
	float fDistortionK1;    /* �������������k1 */
	float fDistortionK2;    /* �������������k2 */
	float fDistortionK3;    /* �������������k3 */
	float fDistortionP1;    /* �������������p1 */
	float fDistortionP2;    /* �������������p2 */
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

	stCamCal.fIntrinsicFx = 1892.602513;	//��ʼ������ڲ�
	stCamCal.fIntrinsicCx = 1437.863993;
	stCamCal.fIntrinsicFy = 1892.602513;
	stCamCal.fIntrinsicCy = 822.3684214;
	stCamCal.fDistortionK1 = -0.286739;
	stCamCal.fDistortionK2 = -0.0337054;
	stCamCal.fDistortionP1 = -0.00005977;
	stCamCal.fDistortionP2 = -0.001538837;
	stCamCal.fDistortionK3 = 0.10722879;

	stCamCal.fDistanceGround = 933;			//��ʼ��������
	stCamCal.fVanishPointX = 1340;
	stCamCal.fVanishPointY = 885;

	int Points[12][2];						//Ŀ��������ʼ������Ŀ����ģ���ṩ����ֵ
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

	Point stDisPoint, *pstDisPoint = &stDisPoint;	//�����Ŀ���(ȥ������ǰ)
	Point stUnDisPoint, *pstUnDisPoint = &stUnDisPoint;	//ȥ�����������ֵ

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

/* ������������������Է����飬ʵ��ȥ������ */
void UnDistortion(Point stDisPoint, Point *pstUnDisPoint, XM_ADAS_CAM_CAL_S stCamCal, float threshold)
{
	float fTargetX, fTargetY;
	float fthresholdX, fthresholdY;
	float fc1, fc2, fTempX, fTempY, fr;
	fc1 = ((float)(stDisPoint.iX) - stCamCal.fIntrinsicCx) / stCamCal.fIntrinsicFx;
	fc2 = ((float)(stDisPoint.iY) - stCamCal.fIntrinsicCy) / stCamCal.fIntrinsicFy;

	fTempX = ((float)(stDisPoint.iX) - stCamCal.fIntrinsicCx) / stCamCal.fIntrinsicFx;	//�������ʼ��
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
