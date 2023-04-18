#include"../include/ClassA.h"

ClassA::ClassA(int iVal)
{
    iValA_ = iVal;
}

ClassA::~ClassA()
{
}

void ClassA::PrintVal()
{
    std::cout<<"ValA:"<<iValA_<<std::endl;
}