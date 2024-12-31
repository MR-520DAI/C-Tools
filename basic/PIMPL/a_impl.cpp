#include "a_interface.h"

class A
{
private:
    int a_;
public:
    A(int data);
    ~A();
    void printA();
};

A::A(int data)
{
    a_ = data;
}

A::~A()
{
}

void A::printA()
{
    std::cout<<a_<<"\n";
}


a_interface::a_interface(int data)
{
    a_pointer = new A(data);
}

a_interface::~a_interface()
{
    delete a_pointer;
    a_pointer = nullptr;
}

void a_interface::printdata()
{
    a_pointer->printA();
}