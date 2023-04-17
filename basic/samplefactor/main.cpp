#include<string>
#include<iostream>

using namespace std;

class Operation
{
public:
    virtual double GetResult(double dValue1, double dValue2)
    {
        dValue1_ = dValue1;
        dValue2_ = dValue2;
        return dValue1_ * dValue2_;
    }
protected:
    double dValue1_;
    double dValue2_;
};

class OperationAdd : public Operation
{
public:
    virtual double GetResult(double dValue1, double dValue2) override
    {
        dValue1_ = dValue1;
        dValue2_ = dValue2;
        return dValue1_ + dValue2_;
    }
};

class OperationSub : public Operation
{
public:
    virtual double GetResult(double dValue1, double dValue2) override
    {
        dValue1_ = dValue1;
        dValue2_ = dValue2;
        return dValue1_ - dValue2_;
    }
};

class SimpleFactor
{
public:
    Operation* Creater(int op)
    {
        Operation* operater = nullptr;
        switch (op)
        {
        case 0:
            operater = new OperationAdd();
            return operater;
        case 1:
            operater = new OperationSub();
            return operater;     
        default:
            cerr<<"参数错误："<<op<<endl;
        }
        return nullptr;
    }
};

int main()
{
    SimpleFactor ft;
    Operation* operater = ft.Creater(0);

    cout<<operater->GetResult(1,1)<<endl;

    delete operater;
    return 0;
}