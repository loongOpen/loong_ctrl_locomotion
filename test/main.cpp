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
#include"main.h"
#include"manager.h"
#include<string>
#include<vector>
#include<map>

#include"log.h"
#include"nabo.h"

double dt=0.001;
Nabo::inputStruct in;
Nabo::outputStruct out;
Ini::iniClass ini("../000.ini");

void setup(){
	// Nabo::init(0.001);
}
void tictoc(){
	// Nabo::step(in,out);

}



