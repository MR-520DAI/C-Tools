#include "../include/objdetect.h"

int main()
{
    int iRet = 0;
    ObjTrack objtrack;

    cv::Mat cvImg = cv::imread("./raw/0000000001.png");

    for(int i = 0; i < 5; i++)
    {
        std::cout<<"******FrameID:"<<i<<"******"<<std::endl;
        iRet = objtrack.detect_yolov5(cvImg);
        draw_objects(cvImg, objtrack.objects_, i);
    }

    return 0;
}