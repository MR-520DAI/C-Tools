#include <string>
#include <iostream>

using namespace std;

class Constant
{
public:
    static Constant* get_instance(){
        static Constant const_variables;
        return &const_variables;
    }

    void print(){
        cout<<"name:"<<name_<<" age:"<<age_<<endl;
        return;
    }
    ~Constant(){
        cout<<"析构\n";
    }
    
private:
    int age_;
    string name_;
    Constant(){
        age_ = 29;
        name_ = "DZY";
    }

    // 浅拷贝复制
    Constant(const Constant&) = default;
    Constant& operator=(const Constant&) = default;
};

int main()
{
    auto c = Constant::get_instance();
    c->print();

    auto b = Constant::get_instance();

    return 0;
}