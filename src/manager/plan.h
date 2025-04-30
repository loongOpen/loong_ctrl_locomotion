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
运动规划基类
继承提供统一接口，可方便做管理调度
继承会导致阅读不便，基类尽量不添加复杂逻辑
=====================================================*/
#pragma once
#include"iopack.h"
#include"robot_config.h"
// #include"robot.h"
#include"nabo_data.h"
#include"log.h"
#include"fsm.h"

namespace Plan{
class basePlanClass{
public:
	basePlanClass();
	virtual void init(float dt);
	virtual void hey(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef)=0;
	virtual bool run(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef)=0;
	virtual void bye(const Nabo::inputStruct &inRef)=0;
	virtual void log()=0;
protected:
/*=================
都会用到的变量，在基类定义。个体plan用到的变量在个体plan中定义即可
==================*/
	// Rbt::rbtClass &rbt=Rbt::rbtClass::instance();
	string planName;//manager内添加到fsm时给定
	float dt;
	float tim;
	bool quitFlag;
	bool logFlag,consoleFlag;
	int logCnt,consoleCnt;

// =====================
// 此处为所有plan收发【常用】的结构体，因此定义了中间变量，在baseRun自动更新
// 【不常用】的数据, 传入直接通过run函数inRef的子元素原地访问即可！
//                传出直接通过outRef的子元素传出即可！
	Nabo::cmdStruct inCmd;
	Nabo::sensorStruct inSens;
	Nabo::ctrlStruct outCtrl;
	Nabo::infoStruct outInfo;
// =====================

	vecNf(NMotMain) j0,jntKp,jntKd;
	Alg::filterOneClass jFil[NMotMain];
	// Alg::filterTwoClass jFil[NMotAll];
	
	Log::logClass logger;
	stringstream logSS;
	// Alg::filterOneClass<float> wFil[NMotAll],tInFil[NMotAll],tOutFil[NMotAll];
	Alg::filterOneClass wFil[NMotMain],tInFil[NMotMain],tOutFil[NMotMain];
	//base开头的函数所有plan都会调用，标记为fianl
	virtual void baseReset(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef)final;
	virtual void baseRun(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef)final;
	virtual const vecNf(NMotMain) &getJntStd()final{return jntStd;}
	virtual const vecNf(NMotMain) &getJntKpInit()final{return jntKpInit;}
	virtual const vecNf(NMotMain) &getJntKdInit()final{return jntKdInit;}
	virtual const vecNf(NMotMain) &getJntMaxTorInit()final{return jntMaxTorInit;}
	virtual const float &getFingerMaxTorInit()final{return fingerMaxTorInit;}
	friend class Mng::fsmClass;
private:
	//防止被意外更改，使用时通过上面的get获取引用
	vecNf(NMotMain) jntStd,jntKpInit,jntKdInit,jntMaxTorInit;
	float fingerMaxTorInit;
};
}//namespace
