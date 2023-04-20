#include<string>
#include<iostream>

using namespace std;

class A
{
private:
    int a_;

public:
    A() {}
    A(int a) : a_(a) {}

    operator string()   // 转换
    {
        return to_string(a_);
    }

    int operator()(int b)   // 重载
    {
        return a_ + b;
    }
};

int main()
{
    A a(10);

    string b = a;
    cout<<b<<endl;

    int c = a(100);

    cout<<c<<endl;

    return 0;
}