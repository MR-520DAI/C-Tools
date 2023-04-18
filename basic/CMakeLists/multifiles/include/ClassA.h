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


#endif  // CLASS_A_H