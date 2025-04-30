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
#include"iopack.h"
#include"plan_sss.h"
#include"timing.h"

namespace Plan{
	freePlanClass::freePlanClass(){
		logFlag=0;
	}
	void freePlanClass::init(float dt){
		basePlanClass::init(dt);
	}
	void freePlanClass::hey(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseReset(inRef,outRef);
		logFlag=0;
	}
	bool freePlanClass::run(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseRun(inRef,outRef);
		outRef.ctrl.enFlag=0;
		outRef.ctrl.j=inRef.sens.j;
		outRef.ctrl.w.setZero();
		outRef.ctrl.t.setZero();

		static int cnt;
		if(cnt%1500==1){
			cout<<"---\n";
			For(NArmDof){
				cout<<inSens.j[i]<<", ";
			}
			cout<<endl;
			For(NArmDof){
				cout<<inSens.j[i+7]<<", ";
			}
			cout<<endl;
			For(NNeckDof+NLumbarDof){
				cout<<inSens.j[i+14]<<", ";
			}
			cout<<endl;
			For(NLegDof){
				cout<<inSens.j[i+MotIdLeg]<<", ";
			}
			cout<<endl;
			For(NLegDof){
				cout<<inSens.j[i+MotIdLeg+6]<<", ";
			}
			cout<<endl;
		}
		cnt++;
		

		if(inRef.cmd.key==1){//en按键
			// outRef.ctrl.enFlag=1;
			// print("free:使能，退出free");
			// return 1;
			// --------------
			int tmp=1;
			For(NMotMain){
				tmp*=inRef.sens.state[i];//非激活=-1，未OP=0
				Alg::clip(tmp, 10);
			}
			if(tmp==0){//只要有一个0，就不能上使能
				print("free：未使能，驱动状态尚未满足要求！");
				printEi("状态字",inRef.sens.state);
			}else{
				outRef.ctrl.enFlag=1;
				print("free:使能，退出free");
				return 1;
			}
		}
		return quitFlag;
	}
	void freePlanClass::log(){
		if(logFlag && inSens.cnt%logCnt==0){
			logSS.clear();logSS.str("");
			logSS<<planName<<"\t"<<tim<<"\t";
			logSS<<"actJ\t";
			For(NMotMain){logSS<<inSens.j[i]<<"\t";}

			logSS<<"rpy\t";
			For3{logSS<<inSens.rpy[i]<<"\t";}
			logSS<<"w\t";
			For3{logSS<<inSens.gyr[i]<<"\t";}
			logSS<<"acc\t";
			For3{logSS<<inSens.acc[i]<<"\t";}

			logSS<<"actW\t";
			For(NMotMain){logSS<<inSens.w[i]<<"\t";}

			logSS<<"actT\t";
			For(NMotMain){logSS<<inSens.t[i]<<"\t";}
			logger.log(logSS.str());
		}

		if(consoleFlag && inSens.cnt%consoleCnt==0){
			cout<<"> "<<planName<<" tim="<<tim<<endl;
		}
	}
// ==========================================
	idlePlanClass::idlePlanClass(){
		Ini::iniClass iniRc("../config/plan_rc.ini");
		ki=iniRc["jntKiAdd"];
		iniRc.getArray("jntKp",jntKp.data(),NMotMain);
		iniRc.getArray("jntKd",jntKd.data(),NMotMain);
	}
	void idlePlanClass::init(float dt){
		basePlanClass::init(dt);
	}
	void idlePlanClass::hey(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseReset(inRef,outRef);
		For(NMotMain){
			Alg::clip(outCtrl.j[i], inSens.j[i]-0.1, inSens.j[i]+0.1);
		}

		cout<<"idle:\nin.j:\n";
		For(NMotMain){
			cout<<inSens.j[i]<<", ";
		}
		cout<<"\nout.j:\n";
		For(NMotMain){
			cout<<outCtrl.j[i]<<", ";
		}
		cout<<endl;
		outCtrl.w.setZero();
		errSum.setZero();
	}
	bool idlePlanClass::run(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseRun(inRef,outRef);
		outRef.param.kp=getJntKpInit();//outRef为引用，单次更改即可
		outRef.param.kd=getJntKdInit();//outRef为引用，单次更改即可
		outRef.param.maxTor=getJntMaxTorInit();//outRef为引用，单次更改即可
		// out维持hey时的值不变
		errSum+=outCtrl.j-inSens.j;
		errSum*=0.99;
		if(inSens.cnt<500){
			float rate=inSens.cnt/500.0;
			outCtrl.t=rate*ki*errSum +(1-rate)*outCtrl.t;
		}else{
			outCtrl.t=ki*errSum;
		}

		For(NMotMain){
			outCtrl.t[i]=tOutFil[i].filt(outCtrl.t[i]);
			Alg::clip(outCtrl.t[i],MaxMotToq);
		}
		outRef.ctrl=outCtrl;
		return quitFlag;
	}
	void idlePlanClass::log(){
		if(consoleFlag && inSens.cnt%consoleCnt==0){
			cout<<"> "<<planName<<" tim="<<tim<<endl;
		}
	}
// ==================================================
	dampPlanClass::dampPlanClass(){
		Ini::iniClass iniBase("../config/plan_base.ini");
		dampRate=iniBase["dampRate"];
		Alg::clip(dampRate,0,1);
	}
	void dampPlanClass::init(float dt){
		basePlanClass::init(dt);
	}
	void dampPlanClass::hey(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseReset(inRef,outRef);
		For(NMotMain){
			Alg::clip(outCtrl.j[i], inSens.j[i]-0.1, inSens.j[i]+0.1);
		}
		outCtrl.w.setZero();
	}
	bool dampPlanClass::run(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseRun(inRef,outRef);
		// out逐渐靠向当前in值
		outCtrl.j=outCtrl.j*dampRate +inSens.j*(1-dampRate);
		outCtrl.t*=0.99;
		if(tim>5){
			outCtrl.enFlag=0;
			if(Mng::fsmClass::instance().enforceSwitchWithoutHighCmd("free", "自动")){
				print("damp 5s 自动下使能！");
			}
		}
		outRef.ctrl=outCtrl;
		return quitFlag;
	}
	void dampPlanClass::log(){
		if(consoleFlag && inSens.cnt%consoleCnt==0){
			cout<<"> "<<planName<<" tim="<<tim<<endl;
		}
	}
// ================================================
}//namespace

