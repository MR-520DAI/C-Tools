#include<memory>
#include<iostream>

using namespace std;

class A
{
public:
    A()
    {
        cout<<"A construct"<<endl;
        throw -1;
    }
    ~A()
    {
        cout<<"A destruct"<<endl;
    }
};

int main()
{
    try
    {
        auto_ptr<A> a(new A);   // 未解决
        cout<<"main A"<<endl;
    }
    catch(int)
    {
        std::cerr << "catched" << '\n';
    }
    
    return 0;
}