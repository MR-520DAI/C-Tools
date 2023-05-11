#ifndef CLASS_A_H
#define CLASS_A_H

#include<iostream>

class ClassA
{
private:
    int iValA_;
public:
    ClassA(int iVal);
    ~ClassA();

    void PrintVal();
};

class HideHpp
{
public:
    HideHpp();
    ~HideHpp();

    void print(int iVal);
private:
    class HideHppImpl;
    class HideHppImpl* ptr;
};

#endif  // CLASS_A_H