#include<iostream>

using namespace std;

class TV
{
private:
    int age1=1;
public:
    int age2=2;
    void printage()
    {
        cout<<"age1:"<<age1<<endl;
        cout<<"age2:"<<age2<<endl;
    }

    friend class Remote;
};

class Remote
{
public:
    void printage1(TV& t)
    {
        t.printage();
    }

    void printage2(TV& t)
    {
        cout<<"age1:"<<t.age1<<endl;
    }
};

int main()
{
    TV t;
    Remote r;
    r.printage1(t);
    r.printage2(t);

    return 0;
}