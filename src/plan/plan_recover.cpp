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
#include"plan_recover.h"

namespace Plan{
	rcPlanClass::rcPlanClass(){
		Ini::iniClass iniRc("../config/plan_rc.ini");
		logFlag=iniRc["logFlag"];
		ki=iniRc["jntKiAdd"];
	#ifdef DefSim
		iniRc.getArray("jntKpSim",jntKp.data(),NMotMain);
		iniRc.getArray("jntKd",jntKd.data(),NMotMain);
	#else
		iniRc.getArray("jntKp",jntKp.data(),NMotMain);
		iniRc.getArray("jntKd",jntKd.data(),NMotMain);
	#endif
		tSit=2;tWait=1;tStand=1;

		// 臂7*2 +头2 +腰3 +腿6*2
		// 0     14   16   19   31
		iniRc.getArray("jntRetract",jntRetract.data(),NMotMain);
	}
	void rcPlanClass::init(float dt){
		basePlanClass::init(dt);
	}
	void rcPlanClass::hey(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseReset(inRef,outRef);
		j0=inSens.j;
		errSum.setZero();
	}
	bool rcPlanClass::run(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseRun(inRef,outRef);
		plan();
		dwdate(outRef);
		return quitFlag;
	}
	void rcPlanClass::plan(){
	#ifdef DefSim
		if(tim<tSit){
			double pgs=tim/tSit;
			double s=0.5-cos(pgs*Pi)*0.5;
			double sd=Pi*0.5*sin(pgs*Pi)/tSit;
			For(NMotMain){
				outCtrl.j[i]=j0[i]+s*(getJntStd()[i]-j0[i]);
				outCtrl.w[i]=sd*(getJntStd()[i]-j0[i]);
			}
		}else{quitFlag=1;}
	#else
		For(NMotMain){
			inSens.w[i]=wFil[i].filt(inSens.w[i]);
		}
		if(tim<tSit){
			double pgs=tim/tSit;
			double s=0.5-cos(pgs*Pi)*0.5;
			double sd=Pi*0.5*sin(pgs*Pi)/tSit;
			For(NMotMain){
				outCtrl.j[i]=j0[i]+s*(jntRetract[i]-j0[i]);
				outCtrl.w[i]=sd*(jntRetract[i]-j0[i]);
			}
		}else if(tim<tSit+tWait){
			For(NMotMain){
				outCtrl.j[i]=jntRetract[i];
				outCtrl.w[i]=0;
			}
		}else if(tim<tSit+tWait+tStand){
			double pgs=(tim-tSit-tWait)/tStand;
			double s=0.5-cos(pgs*Pi)*0.5;
			double sd=Pi*0.5*sin(pgs*Pi)/tStand;
			For(NMotMain){
				outCtrl.j[i]=jntRetract[i]+s*(getJntStd()[i]-jntRetract[i]);
				outCtrl.w[i]=sd*(getJntStd()[i]-jntRetract[i]);
			}
		}else{quitFlag=1;}
	#endif
	}
	void rcPlanClass::dwdate(Nabo::outputStruct &outRef){
		For(NMotMain){
			double err{outCtrl.j[i]-inSens.j[i]};
			errSum[i]+=err;
			errSum[i]*=0.99;
			outCtrl.t[i]=ki*errSum[i];
			outCtrl.t[i]=tOutFil[i].filt(outCtrl.t[i]);
			Alg::clip(outCtrl.t[i],MaxMotToq);
		}
		outRef.ctrl=outCtrl;
	}
	void rcPlanClass::log(){
		if(logFlag && inSens.cnt%logCnt==0){
			logSS.clear();logSS.str("");
			logSS<<planName<<"\t"<<tim<<"\t";
			logSS<<"tgtJ\t";
			For(NMotMain){logSS<<outCtrl.j[i]<<"\t";}
			logSS<<"actJ\t";
			For(NMotMain){logSS<<inSens.j[i]<<"\t";}

			// logSS<<"rpy\t";
			// For3{logSS<<inSens.rpy[i]<<"\t";}
			// logSS<<"w\t";
			// For3{logSS<<inSens.gyr[i]<<"\t";}
			// logSS<<"acc\t";
			// For3{logSS<<inSens.acc[i]<<"\t";}

			logSS<<"tgtW\t";
			For(NMotMain){logSS<<outCtrl.w[i]<<"\t";}
			logSS<<"actW\t";
			For(NMotMain){logSS<<inSens.w[i]<<"\t";}

			logSS<<"tgtT\t";
			For(NMotMain){logSS<<outCtrl.t[i]<<"\t";}
			logSS<<"actT\t";
			For(NMotMain){logSS<<inSens.t[i]<<"\t";}
			logger.log(logSS.str());
		}

		if(consoleFlag && inSens.cnt%consoleCnt==0){
			cout<<"> "<<planName<<" tim="<<tim<<endl;
		}
	}
}//namespace

