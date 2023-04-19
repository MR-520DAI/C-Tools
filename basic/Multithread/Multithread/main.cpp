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
		cout << "唤醒操作" << endl;
		mycond.notify_one();   //唤醒wait
	}
	void MsgRead()
	{
		while (true)
		{
			cout << "进入read" << endl;
			unique_lock<mutex> lck(m);
			cout << "wait阻塞" << endl;
			mycond.wait(lck, [this]() {   //调用wait函数，先解锁lck，然后判断lambda的返回值
				return !msgQue.empty();
			});
			cout << "被唤醒" << endl;
			int ReadData = msgQue.front();  //执行到这里，说明msgQue非空，就可以读取数据了
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