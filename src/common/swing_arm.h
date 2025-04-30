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
#include"eigen.h"
#include"robot_config.h"

namespace Crv{
template<typename T>
class swingArm7Class{
public:
	swingArm7Class(){}
	void setJ0(const vecN(T,NArmDof*2)& j){j0=j;}
	void setDJLeftFront (const vecN(T,NArmDof*2)& dj){djLeftFront=dj;} //左臂前右臂后最大摆臂相对j0的增量
	void setDJRightFront(const vecN(T,NArmDof*2)& dj){djRightFront=dj;}//右臂前左臂后最大摆臂相对j0的增量
	void setMaxDif(T maxDif){this->maxDif=maxDif;}//两腿最大允许的角度差，对应最大摆臂
	
	void reset(){j=j0;};
	void update(const T& dif);//dif=左大腿-右大腿，左腿在前时为正
	const vecN(T,NArmDof*2)& getJ(){return j;}
	const vecN(T,NArmDof*2)& getW(){return w;}
private:
	vecN(T,NArmDof*2) j,w;
	vecN(T,NArmDof*2) j0,djLeftFront,djRightFront;
	T maxDif=0.5;
	T filt=0.05;
};

	template<typename T>
	void swingArm7Class<T>::update(const T& dif){
		T pgs=dif/maxDif;//将大腿夹角映射到0~1
		Alg::clip(pgs,1);
		if(pgs>0){
			j=filt*(pgs*djRightFront +j0) +(1-filt)*j;
		}else{
			pgs=-pgs;
			j=filt*(pgs*djLeftFront +j0) +(1-filt)*j;
		}
	}
}//namespace
