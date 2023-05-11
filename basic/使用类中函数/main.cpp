#include<iostream>
using namespace std;

class A
{
public:
    int a_;
    void static print()
    {
        cout<<"111"<<endl;
    }
};

int main()
{
    A::print();

    return 0;
}