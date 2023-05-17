#include<iostream>
#include<boost/container/vector.hpp>

int main()
{
    boost::container::vector<int> v;
    for (int i = 0; i < 5; i++)
    {
        v.push_back(i);
    }
    
    for (int i = 0; i < 5; i++)
    {
        std::cout<<v.at(i)<<std::endl;
    }
    
    return 0;
}