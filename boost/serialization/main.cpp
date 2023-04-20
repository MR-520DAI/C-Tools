#include<fstream>
#include<sstream>
#include<iostream>
// #include<boost/archive/text_iarchive.hpp>
// #include<boost/archive/text_oarchive.hpp>
#include<boost/archive/binary_oarchive.hpp>
#include<boost/archive/binary_iarchive.hpp>
#include<boost/serialization/base_object.hpp>

class A
{
private:
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar& _tag;
        ar& _text;
    }

public:
    A() {}
    A(int tag, std::string text) : _tag(tag), _text(text) {}
    void printtag()
    {
        std::cout<<_tag<<std::endl;
    }

private:
    int _tag;
    std::string _text;

};

int main()
{
    A a1(2012, "hello");
    std::ostringstream os;

    boost::archive::binary_oarchive oa(os);
    oa << a1;   // 序列化

    std::string content = os.str();
    std::cout<<content<<std::endl;

    A a2;
    std::istringstream is(content);
    boost::archive::binary_iarchive ia(is);
    ia >> a2;   // 反序列化
    a2.printtag();

    return 0;
}