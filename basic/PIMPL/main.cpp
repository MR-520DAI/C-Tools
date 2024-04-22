#include <iostream>
#include "a_interface.h"

using namespace std;

int main()
{
    std::cout<<111<<"\n";

    a_interface a(100);
    a.printdata();

    return 0;
}