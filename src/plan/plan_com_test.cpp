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
#include"plan_com_test.h"

namespace Plan{
	comPlanClass::comPlanClass(){
		testOut.checker=123456;
		testOut.m[0]=12;
		testOut.n[1]=-0.2;
	}
	void comPlanClass::init(float dt){
		basePlanClass::init(dt);
	}
	void comPlanClass::hey(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseReset(inRef,outRef);
		j0=inSens.j;
	}
	bool comPlanClass::run(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseRun(inRef,outRef);


		getTest(inRef);
		plan();

		
		dwdate(outRef);
		return quitFlag;
	}
	void comPlanClass::getTest(const Nabo::inputStruct &inRef){
		if(inRef.app.dataLen==sizeof(testIn)){
			testInStruct*tmp=(testInStruct*)inRef.app.data;
			if(tmp->checker==checker){
				testIn=*tmp;
			}
		}
	}
	void comPlanClass::plan(){
		static int tmp=0;
		if(tmp%500==0){
			print(testIn.a);
			print(testIn.b);
		}
		tmp++;
	}
	void comPlanClass::dwdate(Nabo::outputStruct &outRef){

		outRef.ctrl=outCtrl;
		
		if(testIn.a[0]==9){
			outRef.app.dataLen=sizeof(testOut);
			memcpy(outRef.app.data, &testOut, outRef.app.dataLen);
		}
	}
	void comPlanClass::log(){
		if(logFlag && inSens.cnt%logCnt==0){
			logSS.clear();logSS.str("");
			logSS<<planName<<"\t"<<tim<<"\t";
			logger.log(logSS.str());
		}

		if(consoleFlag && inSens.cnt%consoleCnt==0){
			cout<<"> "<<planName<<" tim="<<tim<<endl;
		}
	}
}//namespace

