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
#pragma once
#include"plan.h"
#include"thread_loop.h"
#include<mutex>
#include<condition_variable>
#include<atomic>
#include"swing_arm.h"
#include<vector>
#include<map>
#include"onnx.h"

namespace Plan{
class rlPlanClass final:public basePlanClass{
public:
	rlPlanClass();
	void init(float dt)override final;
	void hey(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef)override final;
	bool run(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef)override final;
	void bye(const Nabo::inputStruct &inRef)override final{quitingFlag=1;};
	void log() override final;
private:
	bool stopFlag,quitingFlag;
	Thr::threadLoopClass<rlPlanClass> loop;
	Onnx::onnxClass onnx;
	int cntFire;
	atomic<bool> isWait{1};
	mutex mutIn,mutOut;
	condition_variable cv;
	int rlCpuId,rlWaitCnt;
	float rlDt;
	struct obStruct{
		vec3f cmd;
		float z;
		float standFlag;//stand 0.5  step -0.5 walk 0
		vec3f w,g;
		vecNf(12) action,q,qd;
		void setZero(){
			cmd.setZero();
			z=0.42;
			standFlag=0.5;
			w.setZero();
			g<<0,0,-1;
			action.setZero();
			q.setZero();
			qd.setZero();
		}
	}obs,obsTmp,obsNet,obsHist[3];

	struct limitStruct{
		float vx,vy,wz,z;
	}lim;

	vecNf(12) action,actionTmp,actionNet,legQFilt,legQStd;
	int histIdx;

	Crv::swingArm7Class<float> swArmWk;

	void resetObs(Onnx::onnxClass &onnxRef);
	void update();
	bool rlStep();
	void dwdate(Nabo::outputStruct &outRef);
};
}//namespace
