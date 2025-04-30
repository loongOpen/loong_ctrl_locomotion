/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

常用曲线
=====================================================*/
#include"curve.h"
namespace Crv{
//==1维3次曲线，归一化==
	void cubicClass::reset(){
		p0=0;p1=0;
		v0=0;v1=0;
		p=0;v=0;
	}
	void cubicClass::update(double s){
		Alg::clip(s,0,1);
		double s2=s*s,s3=s*s2;
		double a=v1+v0-2*p1+2*p0;
		double b=3*p1-3*p0-2*v0-v1;
		p=a*s3+b*s2+v0*s+p0;
		v=3*a*s2+2*b*s+v0;
	};
//==swing trajectory==
	void swTrj6dClass::setRP1(const mat3d &RR,const vec3d &pp,double T,double pgs){
		pgs=(pgs-0.2)/0.8;
		Alg::clip(pgs,0,1);
		aAxid tmp(R1*RR.transpose());
		R1=aAxid(pgs*tmp.angle(),tmp.axis())*RR;
		For3{
			p1[i]=pp[i] +pgs*(p1[i]-pp[i]);
		}
		this->T=T;
	}
	void swTrj6dClass::update(double s,const double& yaw){
		Alg::clip(s,0,1);
		double dp[3];
		For3{dp[i]=p1[i]-p0[i];}
		double s1=1-s;
		double ss=s*s;

		aAxid dn(R1*R0.transpose());
		R=aAxid((3-s-s)*ss*dn.angle(), dn.axis())*R0;
		w=6*(s-ss)*dn.angle()*dn.axis();
		For2{
			p[i]=p0[i] +(3-s-s)*ss*dp[i];
			v[i]=6*(s-ss)*dp[i];
		}
		p[2]=p0[2] +(16*h*s1*s1 +(3-s-s)*dp[2])*ss;//4次
		v[2]=(32*h*(s1-s)+6*dp[2])*(s-ss);

		w/=T;
		v/=T;
		wv<<w,v;

		// //x方向4次后勾，未考虑方向
		s/=0.9;
		if(s<1){
			s1=1-s;
			ss=16*0.04*s1*s1*s*s;
			p[0]-=ss*cos(yaw);
			p[1]-=ss*sin(yaw);
		}
	}
	void swTrj6dClass::adjust(double z,bool polish){
		if(polish){
			Alg::thresh(z,0.01);
		}
		p[2]-=z*3e-3;
		Alg::clip(p[2],p1[2]-0.08,p1[2]+0.08);
	}
	void swTrj6dClass::adjust(double x,double y,double z,bool polish){
		if(polish){
			Alg::thresh(x,0.01);
			Alg::thresh(y,0.01);
			Alg::thresh(z,0.01);
		}
		p[0]+=x*3e-3;
		p[1]+=y*3e-3;
		p[2]+=z*3e-3;
		Alg::clip(p[0],p1[0]-0.05,p1[0]+0.05);
		Alg::clip(p[1],p1[1]-0.05,p1[1]+0.05);
		Alg::clip(p[2],p1[2]-0.08,p1[2]+0.08);
	}
	void swTrj6dClass::adjust(const vec3d &xyz,bool polish){
		adjust(xyz[0],xyz[1],xyz[2],polish);
	}
	void swTrj6dClass::adjust(const mat3d &R,bool polish){
		aAxid dn(R*this->R.transpose());
		double ang=dn.angle();
		if(polish){
			Alg::thresh(ang,0.01);
		}
		ang*=1e-3;
		this->R=aAxid(ang, dn.axis())*this->R;
	}
}//namespace
