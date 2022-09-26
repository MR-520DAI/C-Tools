#include<iostream>

using namespace std;

typedef struct
{
	short s16X1;        /**< 左上角x坐标 */
	short s16Y1;        /**< 左上角y坐标 */
	short s16X2;        /**< 右下角x坐标 */
	short s16Y2;        /**< 右下角y坐标 */
}XM_IA_RECT_S;

void SdvGetMappingRectRealToRecitified(float* pfIntrinsicK, float* pfDistortion, float* pfIntrinsicP, float* pfRotationMat, XM_IA_RECT_S *pstReal, XM_IA_RECT_S *pstRecitified, int iMaxIteration, float fThred);

int main()
{
	float afDistortionL[5] = { -0.7315388494954922, 2.8361814748657106, 0.010661832399489247, 
		0.00873075165526936, -18.362982236660766 };
	float afIntrinsicKL[9] = { 1751.9756125236643, 0.0, 288.68401375739643, 
		0.0, 1470.2218002641623, 135.5978825494968, 
		0.0, 0.0, 1.0 };
	float afIntrinsicPL[9] = { 1468.019107289771, 0.0, 292.4667339324951, 
		0.0, 1468.019107289771, 133.06743240356445, 
		0.0, 0.0, 1.0 };
	float afRotationMatL[9] = { 0.999984431576014, -0.005366850279604699, -0.001527587533501758, 
		0.005355583649332162, 0.9999591124189039, -0.0072863718113149924, 
		0.0015666299407354521, 0.007278077251171975, 0.9999722872470792 };

	XM_IA_RECT_S stReal;
	XM_IA_RECT_S stRecitified;
	stReal.s16X1 = 299;
	stReal.s16Y1 = 308;
	stReal.s16X2 = 349;
	stReal.s16Y2 = 292;
	stRecitified.s16X1 = 0;
	stRecitified.s16Y1 = 0;
	stRecitified.s16X2 = 0;
	stRecitified.s16Y2 = 0;

	SdvGetMappingRectRealToRecitified((float*)afIntrinsicKL, (float*)afDistortionL,
		(float*)afIntrinsicPL, (float*)afRotationMatL, &stReal, &stRecitified, 20, 0.00001);



	return 0;
}

void SdvGetMappingRectRealToRecitified(float* pfIntrinsicK, float* pfDistortion, float* pfIntrinsicP, float* pfRotationMat, XM_IA_RECT_S *pstReal, XM_IA_RECT_S *pstRecitified, int iMaxIteration, float fThred)
{
	int iIterNum = 0;
	float fc1, fc2;
	float ftempx, ftempy;
	float ftargetx, ftargety;
	float fx_2, fy_2, fr_2, f_2xy, fkr;
	float f_KFx, f_KFy, f_KCx, f_KCy;
	float f_PFx, f_PFy, f_PCx, f_PCy;
	float f_K1, f_K2, f_P1, f_P2, f_K3;

	f_KFx = pfIntrinsicK[0];
	f_KFy = pfIntrinsicK[4];
	f_KCx = pfIntrinsicK[2];
	f_KCy = pfIntrinsicK[5];
	f_PFx = pfIntrinsicP[0];
	f_PFy = pfIntrinsicP[4];
	f_PCx = pfIntrinsicP[2];
	f_PCy = pfIntrinsicP[5];
	f_K1 = pfDistortion[0];
	f_K2 = pfDistortion[1];
	f_P1 = pfDistortion[2];
	f_P2 = pfDistortion[3];
	f_K3 = pfDistortion[4];

	fc1 = ((float)pstReal->s16X1 - f_KCx) / f_KFx;
	fc2 = ((float)pstReal->s16Y1 - f_KCy) / f_KFy;
	ftempx = fc1;
	ftempy = fc2;
	fx_2 = ftempx * ftempx;
	fy_2 = ftempy * ftempy;
	fr_2 = fx_2 + fy_2;
	f_2xy = 2 * ftempx * ftempy;
	fkr = 1 + ((f_K3 * fr_2 + f_K2) * fr_2 + f_K1) * fr_2;
	while (1)
	{
		iIterNum += 1;
		ftargetx = (fc1 - f_P1 * f_2xy + f_P2 * (fr_2 + 2 * fx_2)) / fkr;
		ftargety = (fc2 - f_P1 * (fr_2 + 2 * fy_2) + f_P2 * f_2xy) / fkr;

		fx_2 = ftargetx * ftargetx;
		fy_2 = ftargety * ftargety;
		fr_2 = fx_2 + fy_2;
		f_2xy = 2 * ftargetx * ftargety;
		fkr = 1 + ((f_K3 * fr_2 + f_K2) * fr_2 + f_K1) * fr_2;

		if (abs((fc1 - f_P1 * f_2xy + f_P2 * (fr_2 + 2 * fx_2)) / fkr - ftargetx) < fThred &&
			abs((fc2 - f_P1 * (fr_2 + 2 * fy_2) + f_P2 * f_2xy) / fkr - ftargety) < fThred)
		{
			float f_x, f_y, f_w;
			f_x = ftargety * pfRotationMat[1] + pfRotationMat[2];
			f_y = ftargety * pfRotationMat[4] + pfRotationMat[5];
			f_w = ftargety * pfRotationMat[7] + pfRotationMat[8];

			f_x += ftargetx * pfRotationMat[0];
			f_y += ftargetx * pfRotationMat[3];
			f_w += ftargetx * pfRotationMat[6];

			ftargetx = f_x / f_w * f_PFx + f_PCx;
			ftargety = f_y / f_w * f_PFy + f_PCy;
			pstRecitified->s16X1 = (short)ftargetx;
			pstRecitified->s16Y1 = (short)ftargety;
			break;
		}

		if (iIterNum > iMaxIteration)
		{
			pstRecitified->s16X1 = pstReal->s16X1;
			pstRecitified->s16Y1 = pstReal->s16Y1;
			printf("Iterative solution failed!\n");
			break;
		}
	}

	fc1 = ((float)pstReal->s16X2 - f_KCx) / f_KFx;
	fc2 = ((float)pstReal->s16Y2 - f_KCy) / f_KFy;
	ftempx = fc1;
	ftempy = fc2;
	fx_2 = ftempx * ftempx;
	fy_2 = ftempy * ftempy;
	fr_2 = fx_2 + fy_2;
	f_2xy = 2 * ftempx * ftempy;
	fkr = 1 + ((f_K3 * fr_2 + f_K2) * fr_2 + f_K1) * fr_2;
	while (1)
	{
		iIterNum += 1;
		ftargetx = (fc1 - f_P1 * f_2xy + f_P2 * (fr_2 + 2 * fx_2)) / fkr;
		ftargety = (fc2 - f_P1 * (fr_2 + 2 * fy_2) + f_P2 * f_2xy) / fkr;

		fx_2 = ftargetx * ftargetx;
		fy_2 = ftargety * ftargety;
		fr_2 = fx_2 + fy_2;
		f_2xy = 2 * ftargetx * ftargety;
		fkr = 1 + ((f_K3 * fr_2 + f_K2) * fr_2 + f_K1) * fr_2;

		if (abs((fc1 - f_P1 * f_2xy + f_P2 * (fr_2 + 2 * fx_2)) / fkr - ftargetx) < fThred &&
			abs((fc2 - f_P1 * (fr_2 + 2 * fy_2) + f_P2 * f_2xy) / fkr - ftargety) < fThred)
		{
			float f_x, f_y, f_w;
			f_x = ftargety * pfRotationMat[1] + pfRotationMat[2];
			f_y = ftargety * pfRotationMat[4] + pfRotationMat[5];
			f_w = ftargety * pfRotationMat[7] + pfRotationMat[8];

			f_x += ftargetx * pfRotationMat[0];
			f_y += ftargetx * pfRotationMat[3];
			f_w += ftargetx * pfRotationMat[6];

			ftargetx = f_x / f_w * f_PFx + f_PCx;
			ftargety = f_y / f_w * f_PFy + f_PCy;
			pstRecitified->s16X2 = (short)ftargetx;
			pstRecitified->s16Y2 = (short)ftargety;
			break;
		}

		if (iIterNum > iMaxIteration)
		{
			pstRecitified->s16X2 = pstReal->s16X1;
			pstRecitified->s16Y2 = pstReal->s16Y1;
			printf("Iterative solution failed!\n");
			break;
		}
	}

	return;
}