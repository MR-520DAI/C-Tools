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

class HideHpp::HideHppImpl
{
private:
    /* data */
public:
    HideHppImpl(/* args */){}
    ~HideHppImpl(){}

    void print(int iVal)
    {
        std::cout<<"iVal:"<<iVal<<std::endl;
    }
};


HideHpp::HideHpp()
{
    ptr = new HideHppImpl;
}

HideHpp::~HideHpp()
{
    if (ptr)
    {
        delete ptr;
    }
}

void HideHpp::print(int iVal)
{
    ptr->print(iVal);
}