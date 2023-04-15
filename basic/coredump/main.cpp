#include <iostream>

using namespace std;

int main()
{
    int* p = nullptr;
    *p = 2;     // 程序崩溃
    
    cout<<*p<<endl;
    return 0;
}