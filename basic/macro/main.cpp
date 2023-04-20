#include<iostream>

using namespace std;

int main()
{
#ifdef PRINT_TIME
    cout<<"时间测试"<<endl;
#endif //PRINT_TIME

    cout<<"hello"<<endl;
    
    return 0;
}