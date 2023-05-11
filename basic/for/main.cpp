#include<vector>
#include<iostream>

using namespace std;

int main()
{
    vector<int> v;
    for (int i = 0; i < 5; i++)
    {
        v.push_back(i);
    }
    
    int a = 0;
    for(vector<int>::iterator vit=v.begin(); vit!=v.end(); vit++, a++)
    {
        cout<<"v:"<<(*vit)<<endl;
        cout<<"a:"<<a<<endl;
    }
    return 0;
}