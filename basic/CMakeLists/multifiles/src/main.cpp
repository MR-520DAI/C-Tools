#include"../include/ClassA.h"
#include"../include/ClassB.h"

int main()
{
    ClassA A(5);
    ClassB* B = new ClassB(10);

    A.PrintVal();
    B->PrintVal();

    return 0;
}