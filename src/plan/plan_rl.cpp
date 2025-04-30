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
#include"plan_rl.h"
#include"timing.h"
Timing::timingClass timer;

namespace Plan{
	rlPlanClass::rlPlanClass(){
		Ini::iniClass iniRl("../config/plan_rl.ini");
		logFlag=iniRl["logFlag"];
		iniRl.getArray("legQStd",legQStd.data(),12);
		iniRl.getArray("jntKp",jntKp.data(),NMotMain);
		iniRl.getArray("jntKd",jntKd.data(),NMotMain);

		lim.vx=iniRl["maxVx"];
		lim.vy=iniRl["maxVy"];
		lim.wz=iniRl["maxWz"];
		lim.z=iniRl["maxZ"];

		Ini::iniClass iniSwArm("../config/swing_arm.ini");
		vecNf(NArmDof*2) tmp;
		For(NArmDof*2){tmp[i]=getJntStd()[i];}
		swArmWk.setJ0(tmp);
		iniSwArm.getArray("swingArmRightFront",tmp.data(),NArmDof*2);
		swArmWk.setDJRightFront(tmp);
		iniSwArm.getArray("swingArmLeftFront",tmp.data(),NArmDof*2);
		swArmWk.setDJLeftFront(tmp);
		iniSwArm.getArray("swingArmRunStd",tmp.data(),NArmDof*2);
		iniSwArm.getArray("swingArmRunRightFront",tmp.data(),NArmDof*2);
		iniSwArm.getArray("swingArmRunLeftFront",tmp.data(),NArmDof*2);
		swArmWk.setMaxDif(iniSwArm["walkMaxDif"]);

		string netOnnx=iniRl.getStr("netOnnxs");
		onnx.init("../model/onnx/"+netOnnx, "run");

		Ini::iniClass iniThread("../config/thread.ini");
		rlCpuId=iniThread.getVal("locoAlgorithm", "cpu");
		rlDt=iniThread.getVal("locoAlgorithm", "dt");
	}
	void rlPlanClass::init(float dt){
		basePlanClass::init(dt);
		rlWaitCnt=1/dt*rlDt +1e-6;
	}

	void rlPlanClass::hey(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseReset(inRef,outRef);
		quitingFlag=0;
		stopFlag=1;
		
		resetObs(onnx);
		
		outCtrl.w.setZero();
		outCtrl.t.setZero();
		loop.start("rl", rlCpuId, &rlPlanClass::rlStep, *this);
	}
	void rlPlanClass::resetObs(Onnx::onnxClass &onnxRef){
		cntFire=0;
		obs.setZero();
		obs.w<<inSens.gyr[0],inSens.gyr[1],inSens.gyr[2];
		vec3f g=Ei::rpy2Rf(inSens.rpy).transpose()*vec3f(0,0,-1);
		obs.g<<g[0],g[1],g[2];//double转float
		For(12){
			obs.q[i]=inSens.j[i+MotIdLeg];
		}

		obsTmp=obs;
		obsNet=obs;
		histIdx=0;
		obsHist[0]=obs;
		obsHist[1]=obs;
		obsHist[2]=obs;

		onnxRef.in.setZero();
		onnxRef.in.segment<3>(0)=obs.cmd;
		onnxRef.in[3]=0.42;
		onnxRef.in[4]=obs.standFlag;
		onnxRef.in.segment<3>(5)=obs.w;
		onnxRef.in.segment<3>(8)=obs.g;
		onnxRef.in.segment<12>(11)=obs.action;
		onnxRef.in.segment<12>(23)=obs.q;
		onnxRef.in.segment<12>(35)=obs.qd;
		onnxRef.in.tail<42>()<<onnxRef.in.segment<36>(11), onnxRef.in.segment<6>(5);
		
		For(99){
			onnxRef.in.segment<42>(173+42*i)=onnxRef.in.tail<42>();
		}
		onnxRef.out.setZero();
		action.setZero();
		actionTmp.setZero();
		actionNet.setZero();
		legQFilt=legQStd;
		swArmWk.reset();
	}
	bool rlPlanClass::run(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseRun(inRef,outRef);
		update();
		//rlSetp独立线程调用
		dwdate(outRef);
		return quitFlag;
	}
	void rlPlanClass::update(){
		switch(inCmd.key){
		case -1://掉线
			obs.standFlag=0.5;
			print("掉线");
			break;
		case 6://start踏步
			obs.standFlag=0;
			print("rl start walk");
			break;
		case 7://stop踏步
			obs.standFlag=0.5;
			print("rl stand");
			break;
		}
		
		
		switch(inCmd.naviTap){
		case -1://不导航
			break;
		case 0:
			obs.standFlag=0.5;
			print("rl stand");
			break;
		case 1:
			obs.standFlag=0;
			print("rl start walk");
			break;
		}
		
		if(inSens.cnt>=cntFire && abs(inSens.rpy[0])<0.7 && abs(inSens.rpy[1])<0.7){
			vec3f cmd;
			cmd<<inCmd.vx, inCmd.vy, inCmd.wz;
			if(abs(cmd[0])<0.09){cmd[0]=0;}
			if(abs(cmd[1])<0.04){cmd[1]=0;}
			if(abs(cmd[2])<0.19){cmd[2]=0;}
			
			obs.cmd=0.95*obs.cmd +0.05*cmd;
			obs.z=0.95*obs.z +0.05*(0.42 +inCmd.zOff*10);//不是标准单位
			
			Alg::clip(obs.cmd[0], lim.vx);
			Alg::clip(obs.cmd[1], lim.vy);
			Alg::clip(obs.cmd[2], lim.wz);
			Alg::clip(obs.z, lim.z);
			
			if(obs.cmd.norm()<0.02){
				obs.cmd.setZero();
			}

			obs.w<<inSens.gyr[0],inSens.gyr[1],inSens.gyr[2];
			vec3f g=Ei::rpy2Rf(inSens.rpy).transpose()*vec3f(0,0,-1);
			obs.g<<g[0],g[1],g[2];//double转float
			obs.action=action;
			For(12){
				obs.q[i]=inSens.j[i+MotIdLeg];
				obs.qd[i]=inSens.w[i+MotIdLeg];
			}

			if(isWait){
				unique_lock<mutex> lockIn(mutIn);
				obsTmp=obs;
				isWait=0;
				// print("noti",timer.nowMsUs());
				cv.notify_one();
				lockIn.unlock();
				cntFire=inSens.cnt+rlWaitCnt;//控制网络更新频率
			}
		}

		if(quitingFlag){//rl的退出逻辑还需要修改
			obs.standFlag=0.5;
			if(obs.cmd.norm()<0.01){
				quitFlag=1;
			}
		}
		if(abs(inSens.rpy[0])>1 || abs(inSens.rpy[1])>1){
			print("imu异常", inSens.rpy[0], inSens.rpy[1]);
			Mng::fsmClass::instance().enforceSwitchWithoutHighCmd("damp", "imu倾角过大");
		}
	}
	bool rlPlanClass::rlStep(){//单独线程
		unique_lock<mutex> lockIn(mutIn);
		isWait=1;
		while(isWait){cv.wait(lockIn);}
		obsNet=obsTmp;
		lockIn.unlock();
		
		onnx.in.segment<3>(0)=obsNet.cmd;
		onnx.in[3]=obsNet.z;////
		onnx.in[4]=obsNet.standFlag;
		onnx.in.segment<3>(5)=obsNet.w;
		onnx.in.segment<3>(8)=obsNet.g;
		onnx.in.segment<12>(11)=obsNet.action;
		onnx.in.segment<12>(23)=obsNet.q;
		onnx.in.segment<12>(35)=obsNet.qd;
		For(3){
			int i3=i*3,i12=i*12;
			int idx=(histIdx+i)%3;
			onnx.in.segment<3>(47+i3)=obsHist[idx].w;
			onnx.in.segment<3>(56+i3)=obsHist[idx].g;
			onnx.in.segment<12>( 65+i12)=obsHist[idx].action;
			onnx.in.segment<12>(101+i12)=obsHist[idx].q;
			onnx.in.segment<12>(137+i12)=obsHist[idx].qd;
		}
		onnx.in.segment<4158>(173)=onnx.in.tail<4158>();
		onnx.in.tail<42>()<<onnx.in.segment<36>(11), onnx.in.segment<6>(5);
		// print("run",timer.nowMs(),timer.nowUs());
		onnx.run();
		// print("end",timer.nowMsUs());
		obsHist[histIdx]=obsNet;
		histIdx++;
		histIdx%=3;

		actionNet=onnx.out;
		
		unique_lock<mutex> lockOut(mutOut);
		actionTmp=actionNet;
		lockOut.unlock();
		return 1;
	}
	void rlPlanClass::dwdate(Nabo::outputStruct &outRef){
		unique_lock<mutex> lockOut(mutOut);
		action=actionTmp;
		lockOut.unlock();

		if(abs(inSens.rpy[0])<0.7 && abs(inSens.rpy[1])<0.7){
			legQFilt=0.05*(action+legQStd) +0.95*legQFilt;
		}
		swArmWk.update(inSens.j[MotIdLeg+2] -inSens.j[MotIdLeg+8]);
		
		vecNf(NArmDof*2) swArmJ;
		swArmJ=swArmWk.getJ();
		// (刚进入planer || 切换模型) 后的动作平滑
		if(tim<1){
			float rate=tim;
			For(NArmDof*2){
				outCtrl.j[i]=rate*swArmJ[i] +(1-rate)*outRef.ctrl.j[i];
			}
			For(5){
				outCtrl.j[i+MotIdNeck]=(1-rate)*outRef.ctrl.j[+MotIdNeck];
			}
			For(12){
				outCtrl.j[i+MotIdLeg]=rate*legQFilt[i] +(1-rate)*outRef.ctrl.j[i+MotIdLeg];//与上一次滤波
			}
		}else{
			For(NArmDof*2){
				outCtrl.j[i]=swArmJ[i];
			}
			For(12){
				outCtrl.j[i+MotIdLeg]=legQFilt[i];
			}
		}
		outRef.ctrl=outCtrl;

		// if(tim>1){//测试安全，计时下使能
		// 	Mng::fsmClass::instance().enforceSwitchWithoutHighCmd("damp", "计时安全保护");
		// }
	}
	void rlPlanClass::log(){
		if(logFlag && inSens.cnt%logCnt==0){
			logSS.clear();logSS.str("");
			logSS<<planName<<"\t"<<tim<<"\t";
			
			logSS<<"rlJ\t";
			For(12){logSS<<action[i]<<"\t";}
			logSS<<"tgtJ\t";
			For(12){logSS<<outCtrl.j[i+MotIdLeg]<<"\t";}
			logSS<<"actJ\t";
			For(12){logSS<<inSens.j[i+MotIdLeg]<<"\t";}
			logSS<<"actW\t";
			For(12){logSS<<inSens.w[i+MotIdLeg]<<"\t";}
			logSS<<"rpy\t";
			For3{logSS<<inSens.rpy[i]<<"\t";}

			logger.log(logSS.str());
		}
		if(consoleFlag && inSens.cnt%consoleCnt==0){
			cout<<"> "<<planName<<" tim="<<tim<<endl;
			cout<<"cmd: vx="<<inCmd.vx<<", vy="<<inCmd.vy<<", wz="<<inCmd.wz<<", zOff="<<inCmd.zOff<<endl;
		}
	}
}//namespace
