#pragma once

#include <iostream>
using namespace std;

#ifdef EXPORT_DLL
#define MY_LIB_API __declspec(dllexport)
#else
#define MY_LIB_API __declspec(dllimport)
#endif // EXPORT_DLL

class A;    // 前向声明，重要的实现都包含在类A中

class MY_LIB_API a_interface   // 对用户提供
{
private:
    A* a_pointer;
public:
    a_interface(int data);
    ~a_interface();
    void printdata();
};
