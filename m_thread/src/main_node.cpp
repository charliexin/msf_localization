#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <memory>
#include <condition_variable>
 
using namespace std;
 
 
// 定时器类型
/*
   主要数据：
   1.一个线程变量，保存定时器线程
   2.一个互斥锁，配合条件变量使用
   3.一个条件变量，结合互斥锁，可以是线程不执行任务时，睡眠一段时间，在退出调用时，可以唤醒线程完成退出
   4.定时执行函数，具体的定时执行业务操作
   5.间隔时间，定时器间隔一段时间调用定时执行函数
   6.一个退出标识，标志是否退出定时线程循环
   7.立即执行标识，标识新建状态的定时线程是否立即执行一次任务，而不需等待一个间隔时间才开始执行第一次任务
*/
class CTimer
{
public:
	template<class F>
	CTimer(F func) 
		: m_func(func)
	{}
 
	virtual ~CTimer()
	{}
 
	// 启动函数
	void Start(unsigned int imsec, bool bimmediately_run = false)
	{
		cout << __FUNCTION__ << endl;
		if (imsec == 0 || imsec == static_cast<unsigned int>(-1)) // 间隔时间为0或默认无效值，直接返回
		{
			return;
		}
		m_bexit.store(false);
		m_imsec = imsec;
		m_bimmediately_run.store(bimmediately_run);
		m_thread = std::thread(std::bind(&CTimer::Run,this));
	}
 
	// 结束
	void Stop()
	{
		cout << __FUNCTION__ << endl;
		m_bexit.store(true);
		m_cond.notify_all(); // 唤醒线程
		if (m_thread.joinable())
		{
			m_thread.join();
		}
	}
 
	void SetExit(bool b_exit)
	{
		m_bexit.store(b_exit);
	}
 
private:
	void Run()
	{
		if (m_bimmediately_run.load()) // 立即执行判断
		{
			if (m_func)
			{
				m_func();
			}
		}
 
		while (!m_bexit.load())
		{
			{
				// 锁放在花括号内部，减小锁的粒度
				std::unique_lock<std::mutex> locker(m_mutex);
			
				// 如果是被唤醒的，需要判断先条件确定是不是虚假唤醒
                // wait_for是等待第三个参数满足条件，当不满足时，超时后继续往下执行
				m_cond.wait_for(locker, std::chrono::milliseconds(m_imsec), [this]() { return m_bexit.load(); });
			}
 
			if (m_bexit.load()) // 再校验下退出标识
			{
				return;
			}
 
			if (m_func)
			{
				m_func();
			}
		}
	}
 
private: // 私有数据部分
	std::atomic_bool m_bexit = false;
	std::atomic_bool m_bimmediately_run = false; // 是否立即执行
	unsigned int m_imsec = 1000;	// 间隔时间
	std::function<void()> m_func;	// 执行函数
	std::thread m_thread;
	std::mutex m_mutex;
	std::condition_variable m_cond;
};
 
int g_index = 0;
 
// 定时执行函数
void OnTimeFunc()
{
	cout << __FUNCTION__ << ":" << ++g_index << endl;
}
 
// 测试自己写的定时器
void Test_Timer()
{
	int index = 0;
	std::unique_ptr<CTimer> ptr_timer = std::make_unique<CTimer>(OnTimeFunc);
	if (ptr_timer == nullptr)
	{
		return;
	}
	ptr_timer->Start(1000);
	std::this_thread::sleep_for(std::chrono::seconds(10));
 
	ptr_timer->Stop(); // 离开作用已记得停止定时器
}
 
 
// 程序入口
int main()
{
	Test_Timer();
 
	cout << "getchar" << endl;
	getchar();
	return 0;
}

