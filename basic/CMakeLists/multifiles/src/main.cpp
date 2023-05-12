#include"../include/ClassA.h"
#include"../include/ClassB.h"

int main()
{
    ClassA A(5);
    ClassB* B = new ClassB(10);

    A.PrintVal();
    B->PrintVal();

    HideHpp h;
    h.print(2);

    return 0;
}