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
状态机
“状态”一词易滥用不便区分，此处一个顶层“状态”被称之为一个plan
根据命令key值，自动处理plan之间切换：高优先级命令key直切、普通key通知当前plan退出切、以及用户程序内强切
=====================================================*/
#pragma once
#include"nabo_data.h"
#include<string>

using namespace std;

namespace Plan{
class basePlanClass;
}

namespace Mng{
class fsmClass{
public:
	static fsmClass& instance();

	//首先需添加初始plan（free plan）
	void addInit(Plan::basePlanClass &plan,const int& key,const string& name="free",bool enforce=1);
	//其次需添加空闲plan。某plan退出但未指定下一plan时，将自动进入空闲plan
	void addIdle(Plan::basePlanClass &plan,const int& key,const string& name="idle",bool enforce=1);
	//然后再添加其他plan
	//enforce=1：当需切换本plan时，无需等待正在运行的plan退出，强制转换为本plan；
	//enforce=0：当需切换本plan时，正在运行的plan退出后，转换为本plan；
	void addCommon(Plan::basePlanClass &plan,const int& key,const string& name,bool enforce);

	void init(float dt);
	void step(Nabo::inputStruct &in,Nabo::outputStruct &out);
	const string &whoNow();

	//==强切：用于在plan中手动强制切换plan，谨慎使用(所以函数名很长!)。成功返回1，失败返回0
	// 使用方式：Mng::fsmClass::instance().enforceSwitchWithoutHighCmd()
	bool enforceSwitchWithoutHighCmd(const int& tgtPlanKey, const string &switchReason);
	bool enforceSwitchWithoutHighCmd(const string& tgtPlanName, const string &switchReason);
private:
	fsmClass();
	fsmClass(const fsmClass&)=delete;
	fsmClass & operator=(const fsmClass&)=delete;
	
	class impClass;
	impClass &imp;
};
}//namespace
