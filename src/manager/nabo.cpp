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
本层仅用于C风格封装，不做任何算法业务
添加SIGSEGV信号处理，用于辅助debug
=====================================================*/
#include"manager.h"
#include"nabo.h"
#include<signal.h>
// #include"timing.h"
#include<iostream>
#include<execinfo.h>

using namespace std;

void doTrace(int sig){
	const int size=256;
	void *buffer[size];
	char **strings;
	
	int nptrs=backtrace(buffer, size);
	strings=backtrace_symbols(buffer, nptrs);
	cout<<"===错误堆栈===\n";
	if(strings){
		for(int i=0;i<nptrs;++i){
			cout<<strings[i]<<endl;
		}
		free(strings);
	}
	cout<<"============\n";
	signal(sig,SIG_DFL);
}

namespace Nabo{
	void init(float dt){
		signal(SIGSEGV, doTrace);
		Mng::manageClass::instance().init(dt);
	}
	void step(inputStruct &in,outputStruct &out){
		Mng::manageClass::instance().step(in,out);
	}
}//namespace
