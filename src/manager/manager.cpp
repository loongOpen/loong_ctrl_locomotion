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
#include"manager.h"
#include"plan_sss.h"
#include"plan_recover.h"
#include"plan_com_test.h"
#include"plan_rl.h"
#include"timing.h"
#include"fsm.h"

enum KeyEnum:int{
	KeyY=1,
	KeyX=2,
	KeyA=3,
	KeyB=4,
	KeyLL=6,
	KeyRR=7,
	KeyUp=10,
	KeyRight=11,
	KeyLeft=12,
	KeyDown=13,
};
float jArmNeck[16]{ 0.3,-1.3, 1.8, 0.5, 0,0,0,
					-0.3,-1.3,-1.8, 0.5, 0,0,0,
					0,0};
// ================================
namespace Mng{
class manageClass::impClass{
public:
	impClass();
	Plan::freePlanClass free;
	Plan::idlePlanClass idle;
	Plan::dampPlanClass damp;
	// Plan::jntSdkPlanClass jntSdk;
	Plan::rcPlanClass rc;
	Plan::rlPlanClass rl;
	Plan::comPlanClass test;
	fsmClass &fsm=fsmClass::instance();

	void step(Nabo::inputStruct &in,Nabo::outputStruct &out);
};
	manageClass::impClass::impClass(){
		// ------------------
		// 首先addInit添加初始plan
		// 其次addIdle添加空闲plan。某plan退出但未指定下一plan时，将自动进入空闲plan
		// 然后addCommon再添加其他plan
		// enforce=1：当需切换本plan时，无需等待正在运行的plan退出，强制转换为本plan；
		// enforce=0：当需切换本plan时，正在运行的plan退出后，转换为本plan；
		// ------------------
#ifdef DefSim //仿真简化，借用fms idle的自动特性，直接切入待测试plan
		fsm.addInit(rc,100,"rc",1);
		// fsm.addIdle(nb,101,"nb",1);
		fsm.addIdle(rl,4,"rl",0);
		// fsm.addIdle(idle,KeyA);
		fsm.addCommon(free,KeyDown,"free",1);
		fsm.addCommon(damp,20,"damp",1);
#else
		fsm.addInit(free,KeyDown);
		fsm.addIdle(idle,KeyA);
		fsm.addCommon(damp,KeyLeft,"damp",1);
		fsm.addCommon(rc,KeyX,"rc",0);
		fsm.addCommon(rl,KeyB,"rl",0);
		// fsm.addCommon(jntSdk,23,"jntSdk",0);
		// fsm.addCommon(test,22,"test",0);
#endif
	}
	void manageClass::impClass::step(Nabo::inputStruct &in,Nabo::outputStruct &out){
#ifndef DefSim
		int tmp=1;
		For(14){
			tmp*=in.sens.state[i];
			Alg::clip(tmp,10);
		}
		if(!tmp){//判断上肢是否开启
			memcpy(in.sens.j.data(), jArmNeck, 16*4);
			memset(in.sens.w.data(), 0, 16*4);
			memset(in.sens.t.data(), 0, 16*4);
		}
#endif
		fsm.step(in,out);

		// 报错则自动阻尼
		int err=0;
		if(out.ctrl.enFlag){
			For(NMotLeg){
				err|=in.sens.state[i+MotIdLeg]&8;//报错时bit3为1
			}
			if(err){
				fsm.enforceSwitchWithoutHighCmd("damp", "某电机报错");
			}
		}
	}
//=====================================================
	manageClass::manageClass():imp(*new impClass()){}
	manageClass& manageClass::instance(){
		static manageClass singtn;
		return singtn;
	}
	void manageClass::init(float dt){
		imp.fsm.init(dt);
		cout<<"manager: dt="<<dt<<endl;
	}
	void manageClass::step(Nabo::inputStruct &in,Nabo::outputStruct &out){
		imp.step(in,out);
	}
}//namespace
