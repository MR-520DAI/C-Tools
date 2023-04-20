#include <string>
#include <iostream>

using namespace std;

/* 异常基类 */
class MyException
{
public:
	MyException(const string& errorMsg, int iId)
		:errorMsg_(errorMsg), iId_(iId)
	{}
 
	virtual string what() const
	{
		return errorMsg_;
	}
 
protected:
	int iId_;
	string errorMsg_;

};

class MyException1 : public MyException
{
public:
    MyException1(const string& errorMsg, int iId, const string& errorMsg1) :MyException(errorMsg, iId),
    errorMsg1_(errorMsg1) {}

    virtual string what() const override
    {
        string errorMsg = "MyException1: ";
        errorMsg += errorMsg_;
        errorMsg += "-->";
        errorMsg += errorMsg1_;
        errorMsg += "-->";
        errorMsg += to_string(iId_);
        return errorMsg;
    }
protected:
    string errorMsg1_;
};

void CreateException(int value1, int value2)
{
    if (value1 == value2)
    {
        throw MyException1("error", value1, "MyException1");
    }
    else
    {
        cout<<"value1: "<<value1<<endl;
    }
    return;
}

int main()
{
    int value1 = 10;
    int value2 = 10;

    try
    {
        CreateException(value1, value2);
    }
    catch(const MyException& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    return 0;
}