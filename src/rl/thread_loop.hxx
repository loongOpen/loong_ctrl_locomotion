/*
Copyright 2025 国家地方共建人形机器人创新中心/人形机器人（上海）有限公司, https://www.openloong.net
Thanks for the open biped control project Nabo: https://github.com/tryingfly/nabo

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

============ ***doc description @ yyp*** ============

=====================================================*/
#include"thread_loop.h"
#include<thread>
#include<atomic>
#include<sstream>
#include"iopack.h"
#include"timing.h"

// #define DefLoopTiming

namespace Thr{

template<class T>
class threadLoopClass<T>::impClass{
public:
	void loop();
	bool (T::*runOnce)();
	T*t;
	string name;

	int cpuId;
	atomic<bool> isRun{0};
	Timing::timingClass timer1,timer2;
	int printCnt;
};

	template<class T>
	void threadLoopClass<T>::impClass::loop(){
		cpu_set_t mask;
		CPU_ZERO(&mask);
		CPU_SET(cpuId,&mask);
		stringstream ss;
		if(pthread_setaffinity_np(pthread_self(),sizeof(mask),&mask)<0){
			ss<<name<<"线程绑定cpu失败";;
			throw runtime_error(ss.str());
		}else{
			CPU_ZERO(&mask);
			if(pthread_getaffinity_np(pthread_self(),sizeof(mask),&mask)<0){
				ss<<name<<"线程获取绑定cpu失败";
				throw runtime_error(ss.str());
			}else{
				int cpuNum=thread::hardware_concurrency();
				For(cpuNum){
					if(CPU_ISSET(i,&mask)){printL(name,"线程绑定到cpu-",i);break;}
				}
			}
		}
		while(isRun){
#ifdef DefLoopTiming
			timer1.tic();
#endif

			isRun=(t->*runOnce)();

			//==平均耗时计算（20个点）==
#ifdef DefLoopTiming
			if(printCnt%20==0){
				printL(name,"线程计算平均耗时",timer1.toc20(),"ms，循环平均耗时",timer2.toc20(),"ms");
			}else{
				timer1.toc20();
				timer2.toc20();
			}
			printCnt++;
			timer2.tic();
#endif
		}
	}
//==================================================
	template<class T>
	threadLoopClass<T>::threadLoopClass():imp(*new impClass()){}
	template<class T>
	threadLoopClass<T>::~threadLoopClass(){
		imp.isRun=0;
	}

	template<class T>
	bool threadLoopClass<T>::start(const string &name, int cpuId, bool(T::*fun)(), T&t){
		if(!imp.isRun){
			imp.isRun=1;
			imp.cpuId=cpuId;
			imp.runOnce=fun;
			imp.t=&t;
			imp.name=name;
			thread thd(&threadLoopClass<T>::impClass::loop,&(this->imp));

			// // 设置调度策略和优先级
			// pthread_t pthreadId=thd.native_handle();
			// sched_param schedParam;
			// schedParam.sched_priority=80;
			// if(pthread_setschedparam(pthreadId, SCHED_FIFO, &schedParam)!=0){
			// 	cerr<<name<<"线程优先级设定失败"<<endl;
			// }

			thd.detach();
			return 1;
		}
		return 0;
	}
	template<class T>
	bool threadLoopClass<T>::isAlive(){
		return imp.isRun;
	}
}//namespace

