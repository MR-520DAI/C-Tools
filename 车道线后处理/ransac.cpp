#include<cmath>
#include<iostream>
using namespace std;

typedef struct
{
    int iNum;
    float afPoints[100][2];
}XM_ADAS_LANE_POINTS;

XM_ADAS_LANE_POINTS RansacFit(XM_ADAS_LANE_POINTS stLanePoint, int iIterrations, float fthred)
{
    float afline[2][2];
    int iBestScore = -1;
    srand((unsigned)time(NULL));
    int iCount = stLanePoint.iNum;
    XM_ADAS_LANE_POINTS stLanefitRslt;
    memset(&stLanefitRslt, 0, sizeof(XM_ADAS_LANE_POINTS));

    for (int k = 0; k < iIterrations; k++)
    {
        /* 随机选取两个点索引 */
        int iIndex1 = 0, iIndex2 = 0;
        while (iIndex1 == iIndex2)
        {
            iIndex1 = rand() % iCount;
            iIndex2 = rand() % iCount;
        }
        /* 随机选取两点建立直线模型 */
        float afp1[2] = { stLanePoint.afPoints[iIndex1][0], stLanePoint.afPoints[iIndex1][1] };
        float afp2[2] = { stLanePoint.afPoints[iIndex2][0], stLanePoint.afPoints[iIndex2][1] };

        float afv1[2] = { afp2[0] - afp1[0], afp2[1] - afp1[1] };
        float flength = sqrt(afv1[0] * afv1[0] + afv1[1] * afv1[1]);
        afv1[0] /= flength;
        afv1[1] /= flength;

        int iscore = 0;
        for (int i = 0; i < iCount; i++)
        {
            float afp3[2] = { stLanePoint.afPoints[i][0], stLanePoint.afPoints[i][1] };
            float afv2[2] = { afp3[0] - afp1[0], afp3[1] - afp1[1] };
            flength = sqrt(afv2[0] * afv2[0] + afv2[1] * afv2[1]);
            afv2[0] /= flength;
            afv2[1] /= flength;
            float fd = afv2[1] * afv1[0] - afv2[0] * afv1[1];

            if (fabs(fd) < fthred)
            {
                iscore++;
            }
        }
        if (iscore > iBestScore)
        {
            afline[0][0] = afp1[0];
            afline[0][1] = afp1[1];
            afline[1][0] = afp2[0];
            afline[1][1] = afp2[1];
            iBestScore = iscore;
        }
    }

    float afbestv[2] = { afline[1][0] - afline[0][0], afline[1][1] - afline[0][1] };
    float flength = sqrt(afbestv[0] * afbestv[0] + afbestv[1] * afbestv[1]);
    afbestv[0] /= flength;
    afbestv[1] /= flength;

    for (int i = 0; i < iCount; i++)
    {
        float afp3[2] = { stLanePoint.afPoints[i][0], stLanePoint.afPoints[i][1] };
        float afv2[2] = { afp3[0] - afline[0][0], afp3[1] - afline[0][1] };
        flength = sqrt(afv2[0] * afv2[0] + afv2[1] * afv2[1]);
        afv2[0] /= flength;
        afv2[1] /= flength;
        float fd = afv2[1] * afbestv[0] - afv2[0] * afbestv[1];
        if (fabs(fd) < fthred)
        {
            stLanefitRslt.afPoints[stLanefitRslt.iNum][0] = afp3[0];
            stLanefitRslt.afPoints[stLanefitRslt.iNum][1] = afp3[1];
            stLanefitRslt.iNum++;
        }
    }
    return stLanefitRslt;
}

int main()
{
    XM_ADAS_LANE_POINTS stLanePoints;
    float data[] = {
        456.000000,160.000000,
512.000000,296.000000,
520.000000,296.000000,
512.000000,304.000000,
520.000000,304.000000,
528.000000,304.000000,
528.000000,312.000000,
536.000000,312.000000,
544.000000,312.000000,
544.000000,320.000000,
552.000000,320.000000,
552.000000,328.000000,
560.000000,328.000000,
568.000000,328.000000,
568.000000,336.000000
    };
    int len = sizeof(data) / sizeof(data[0]);
    stLanePoints.iNum = 0;
    for (int i = 0; i < len; i+=2)
    {
        stLanePoints.afPoints[stLanePoints.iNum][0] = data[i];
        stLanePoints.afPoints[stLanePoints.iNum][1] = data[i + 1];
        stLanePoints.iNum++;
    }

    stLanePoints = RansacFit(stLanePoints, 20, 0.087);

    for (int i = 0; i < stLanePoints.iNum; i++)
    {
        cout << stLanePoints.afPoints[i][0] << "," << stLanePoints.afPoints[i][1] << "," << endl;
    }

    return 0;
}