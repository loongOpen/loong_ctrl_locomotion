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
#include"plan.h"
#include"unistd.h"

namespace Plan{
	basePlanClass::basePlanClass(){
		consoleFlag=1;
		Ini::iniClass iniBase("../config/plan_base.ini");
		logFlag=iniBase["logFlag"];
		iniBase.getArray("jntStd",jntStd.data(),NMotMain);
		iniBase.getArray("jntKpInit",jntKpInit.data(),NMotMain);
		iniBase.getArray("jntKdInit",jntKdInit.data(),NMotMain);
		jntKp=jntKpInit;
		jntKd=jntKdInit;
		iniBase.getArray("jntMaxTorInit",jntMaxTorInit.data(),NMotMain);
		fingerMaxTorInit=iniBase["fingerMaxTorInit"];
		logger.init("loco");
	}
	void basePlanClass::init(float dtt){
		dt=dtt;
		// rbt.setDt(dt);
		logCnt=int(0.01/dt+0.0001);//0.01s一帧log
		consoleCnt=int(1/dt+0.0001);//一帧控制台print
	}
	void basePlanClass::baseReset(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		tim=0;
		quitFlag=0;
		inCmd=inRef.cmd;
		inSens=inRef.sens;
		outCtrl=outRef.ctrl;
		outRef.param.kp=jntKp;
		outRef.param.kd=jntKd;
		outRef.param.maxTor=getJntMaxTorInit();
		For(NMotMain){
			wFil[i].init(dt,20,0);
#ifdef DefSim
			tOutFil[i].init(dt,50,0);
#else
			tOutFil[i].init(dt,50,0,outCtrl.t[i]);
#endif
		}
		// rbt.imu.init(inSens.rpy);
		// rbt.setSwing(0,0);
		// rbt.setSwing(1,0);
		cout<<planName<<" plan hey!\n";
	}
	void basePlanClass::baseRun(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		inCmd=inRef.cmd;
		inSens=inRef.sens;
		tim=inSens.cnt*dt;
	}
}//namespace

