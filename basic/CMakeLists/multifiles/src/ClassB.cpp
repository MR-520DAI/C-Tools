#include"../include/ClassB.h"

ClassB::ClassB(int iVal)
{
    iValB_ = iVal;
}

ClassB::~ClassB()
{
}

void ClassB::PrintVal()
{
    std::cout<<"ValB:"<<iValB_<<std::endl;
}