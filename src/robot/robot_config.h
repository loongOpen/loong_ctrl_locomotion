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
常量定义，首字母大写
#include"robot_config.h"
=====================================================*/
#pragma once
//涉及数组大小，必须编译前给定
static const int NLegDof       =6;
static const int NArmDof       =7;
static const int NLumbarDof    =3;
static const int NNeckDof      =2;

static const int NMotLeg       =NLegDof*2;
static const int NMotArm       =NArmDof*2;
static const int NMotNeck      =NNeckDof;
static const int NMotLumbar    =NLumbarDof;
static const int NMotMain      =NMotLeg +NMotArm +NNeckDof +NLumbarDof;


static const int NBaseDof      =6;
static const int NGenDof       =NBaseDof +NMotMain;
//其他常量
static const double HipY       =0.12;
static const double HipZ       =-0.167;
static const double ThighOffZ  =-0.1155;
static const double LenThigh   =0.4;
static const double LenShank   =0.387;
static const double BodyH      =1.02;
static const double MaxMotToq  =300;
static const double MaxFz      =1200;

const int QdIdNeck=20;
const int QdIdLumbar=22;
const int QdIdHip[2]{25,31};
const int QdIdArm[2]{6,13};
const int QdIdWrist[2]{10,17};
const int QdIdKnee[2]{28,34};

const int MotIdNeck=14;
const int MotIdLumbar=16;
const int MotIdLeg=19;
