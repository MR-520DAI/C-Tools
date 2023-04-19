#include<deque>
#include<mutex>
#include<thread>
#include<iostream>
#include<Windows.h>
#include<condition_variable>

using namespace std;

class Msg
{
private:
	int seq;
	deque<int> msgQue;
	mutex m;
	condition_variable mycond;
public:
	void MsgWrite()
	{
		seq = 0;
		while (true)
		{
			unique_lock<mutex> lck(m);
			msgQue.push_back(seq);
			cout << "Write Message : " << seq << endl;
			++seq;
			if (seq == 10)
			{
				break;
			}
		}
		cout << "���Ѳ���" << endl;
		mycond.notify_one();   //����wait
	}
	void MsgRead()
	{
		while (true)
		{
			cout << "����read" << endl;
			unique_lock<mutex> lck(m);
			cout << "wait����" << endl;
			mycond.wait(lck, [this]() {   //����wait�������Ƚ���lck��Ȼ���ж�lambda�ķ���ֵ
				return !msgQue.empty();
			});
			cout << "������" << endl;
			int ReadData = msgQue.front();  //ִ�е����˵��msgQue�ǿգ��Ϳ��Զ�ȡ������
			msgQue.pop_front();
			cout << "Read Message : " << ReadData << endl;
			if (ReadData == 9)
			{
				break;
			}
		}
	}
};

int main()
{
	Msg myMsg;
	thread wt(&Msg::MsgWrite, &myMsg);
	thread rt(&Msg::MsgRead, &myMsg);
	Sleep(1000);
	

	rt.join();
	wt.join();

	return 0;
}