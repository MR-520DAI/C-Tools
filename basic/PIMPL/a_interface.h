#pragma once

#include <iostream>
using namespace std;

class A;    // 前向声明，重要的实现都包含在类A中
class a_interface   // 对用户提供
{
private:
    A* a_pointer;
public:
    a_interface(int data);
    ~a_interface();
    void printdata();
};
