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
#include"fsm.h"
#include<vector>
#include<map>
#include<string>
#include"plan.h"
#include"iopack.h"
#include"timing.h"

namespace Mng{
class fsmClass::impClass{
public:
	impClass(fsmClass*omp);
	void add(Plan::basePlanClass &plan,const int& key,const string& name,bool enforce);
	void init(float dt);
	void step(Nabo::inputStruct &in,Nabo::outputStruct &out);
	bool enforceSwitch(const int &tgtPlanKey, const string &switchReason);
	bool enforceSwitch(const string & tgtPlanName, const string &switchReason);
	void onErr(stringstream&ss);
	
	fsmClass&omp;
	Timing::timingClass timer;

	map<int,int> keyIds;
	map<string,int> nameIds;
	struct storeStruct{
		storeStruct(int key, Plan::basePlanClass*p, const string&n, bool f):key(key), plan(p), name(n), enforce(f){}
		int key;
		Plan::basePlanClass* plan;
		string name;
		bool enforce;
	};
	vector<storeStruct> store;

	float dtWarn;
	int size,id,idNext,keyOld;
	bool switchFlag,firstFlag;
	Plan::basePlanClass *plan,*planNext;
};
	fsmClass::impClass::impClass(fsmClass*omp):omp(*omp){
		size=0;
		id=0;
		idNext=-1;
		keyOld=-1;
		store.reserve(16);
	}
	void fsmClass::impClass::onErr(stringstream&ss){
		print(ss.str());
		throw runtime_error(ss.str());
		exit(1);
	}
	void fsmClass::impClass::add(Plan::basePlanClass &plan,const int& key,const string& name,bool enforce){
		if(name.size()>15){
			stringstream ss;
			ss<<"fsm: 所add的plan其name长度不可>15，name="<<name;
			onErr(ss);
		}
		if(key<1){
			stringstream ss;
			ss<<"fsm: add()的key不可<1!";
			onErr(ss);
		}
		auto it=keyIds.find(key);
		if(it!=keyIds.end()){
			stringstream ss;
			ss<<"fsm: add()的key重复,key="<<key;
			onErr(ss);
		}
		auto itt=nameIds.find(name);
		if(itt!=nameIds.end()){
			stringstream ss;
			ss<<"fsm: add()的name重复,name="<<name;//name需要enforce switch用，不能重复
			ss<<"\n>> 若需要为同一plan添加不同key而产生name重复，可将第二次添加的name稍作修改！\n===\n";
			onErr(ss);
		}
		keyIds.insert(pair<int,int>(key,size));
		nameIds.insert(pair<string,int>(name,size));
		store.emplace_back(key,&plan,name,enforce);
		plan.planName=name;
		size++;
	}
	void fsmClass::impClass::init(float dt){
		dtWarn=1000*dt*0.9;
		for(auto &it:store){
			it.plan->init(dt);
		}
		plan=store[0].plan;
		planNext=plan;
		id=0;
		idNext=-1;
		switchFlag=1;
		firstFlag=1;
	}
	void fsmClass::impClass::step(Nabo::inputStruct &in,Nabo::outputStruct &out){
		if(firstFlag){
			keyOld=in.cmd.key;
			firstFlag=0;
		}
		if(in.cmd.key!=keyOld){
			keyOld=in.cmd.key;
		}else{
			in.cmd.key=0;
		}
		if(in.cmd.key>0){
			auto it=keyIds.find(in.cmd.key);
			if(it!=keyIds.end()){
				int tmpId=it->second;
				if(tmpId!=id){
					if(store[tmpId].enforce){
						printL(">> fms:强制 ",store[id].name," 切换为 ",store[tmpId].name);
						planNext=store[tmpId].plan;
						id=tmpId;
						switchFlag=1;
					}else{
						printL(">> fsm:通知 ",store[id].name," 请求切换 ",store[tmpId].name);
						plan->bye(in);
						idNext=tmpId;
					}
				}
			}
		}
		timer.tic();
		if(switchFlag){
			in.sens.cnt=0;
			plan=planNext;
			switchFlag=0;
			printL(">> fms:进入 ",store[id].name);
			plan->hey(in,out);
			out.info.planKey=store[id].key;
			strcpy(out.info.planName, store[id].name.data());
		}
		if(plan->run(in,out)){
			if(idNext==-1){
				idNext=1;
			}
			printL(">> fms:正在退出 ",store[id].name,"，将进入 ",store[idNext].name);
			id=idNext;
			planNext=store[id].plan;
			idNext=-1;
			switchFlag=1;
		}
		float ms;
		ms=timer.toc();
		if(ms>dtWarn){print("run超时",ms,"ms");}
		timer.tic();
		plan->log();
		ms=timer.toc();
		if(ms>dtWarn){print("log超时",ms,"ms");}
		in.sens.cnt++;
	}
	bool fsmClass::impClass::enforceSwitch(const int &tgtPlanKey, const string &switchReason){
		auto it=keyIds.find(tgtPlanKey);
		if(it!=keyIds.end()){
			int tmpId=it->second;
			if(tmpId!=id){
				printL(">> fms:程序内强制 ",store[id].name," 切换为 ",store[tmpId].name, "。原因：", switchReason);
				planNext=store[tmpId].plan;
				id=tmpId;
				switchFlag=1;
				return 1;
			}
		}
		return 0;
	}
	bool fsmClass::impClass::enforceSwitch(const string &tgtPlanName, const string &switchReason){
		auto it=nameIds.find(tgtPlanName);
		if(it!=nameIds.end()){
			int tmpId=it->second;
			if(tmpId!=id){
				printL(">> fms:程序内强制 ",store[id].name," 切换为 ",store[tmpId].name, "。原因：", switchReason);
				planNext=store[tmpId].plan;//本plan已执行，还差log，下一周期切换
				id=tmpId;
				switchFlag=1;
				return 1;
			}
		}
		return 0;
	}
// =======================================
	fsmClass::fsmClass():imp(*new impClass(this)){}
	fsmClass& fsmClass::instance(){
		static fsmClass singtn;
		return singtn;
	}
	void fsmClass::addInit(Plan::basePlanClass &plan,const int& key,const string& name,bool enforce){
		if(imp.size!=0){
			stringstream ss;
			ss<<"fsm: addInit 必须首先被调用！";
			imp.onErr(ss);
		}
		imp.add(plan,key,name,enforce);
		imp.plan=&plan;
		imp.planNext=&plan;
	}
	void fsmClass::addIdle(Plan::basePlanClass &plan,const int& key,const string& name,bool enforce){
		if(imp.size!=1){
			stringstream ss;
			ss<<"fsm: addIdle 必须在 addInit 后被调用！";
			imp.onErr(ss);
		}
		imp.add(plan,key,name,enforce);
	}
	void fsmClass::addCommon(Plan::basePlanClass &plan,const int& key,const string& name,bool enforce){
		if(imp.size<2){
			stringstream ss;
			ss<<"fsm: 必须先调用 addInit、addIdle！";
			imp.onErr(ss);
		}
		imp.add(plan,key,name,enforce);
	}
	const string &fsmClass::whoNow(){
		return imp.store[imp.id].name;
	}
	void fsmClass::init(float dt){
		imp.init(dt);
	}
	void fsmClass::step(Nabo::inputStruct &in,Nabo::outputStruct &out){
		imp.step(in,out);
	}
	bool fsmClass::enforceSwitchWithoutHighCmd(const int &tgtPlanKey, const string &switchReason){
		return imp.enforceSwitch(tgtPlanKey, switchReason);
	}
	bool fsmClass::enforceSwitchWithoutHighCmd(const string &tgtPlanName, const string &switchReason){
		return imp.enforceSwitch(tgtPlanName, switchReason);
	}
}//namespace
