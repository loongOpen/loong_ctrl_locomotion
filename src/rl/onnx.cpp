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
#include"onnx.h"
#include"iopack.h"
#include"onnxruntime/onnxruntime_cxx_api.h"
#include<sstream>

namespace Onnx{
class onnxClass::impClass{
public:
	impClass(onnxClass*omp);
	bool init(const string& onnxFile, const string &name);
	void run();
	onnxClass&omp;
	string name;

	Ort::Session *ort=nullptr;
	string inName,outName;
	char*inNamePtr, *outNamePtr;
	Ort::Value inTensor, outTensor;
	float* inPtr, *outPtr;
	int inSize, outSize;
	Ort::RunOptions runOpts;
};
	onnxClass::impClass::impClass(onnxClass*omp):omp(*omp),runOpts(nullptr){

	}
	bool onnxClass::impClass::init(const string& onnxFile, const string &name){
		this->name=name;
		Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "loong");
		Ort::SessionOptions seOptions;
		seOptions.SetInterOpNumThreads(1);
		seOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
		if(!ort){
			ort=new Ort::Session(env, onnxFile.c_str(), seOptions);
		}

		Ort::AllocatorWithDefaultOptions allocator;
		inName=ort->GetInputNameAllocated(0,allocator).get();
		outName=ort->GetOutputNameAllocated(0,allocator).get();

		auto inShape=ort->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
		auto outShape=ort->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
		
		inSize=inShape[0]*inShape[1];
		outSize=outShape[0]*outShape[1];

		omp.in.setZero(inSize);
		omp.out.setZero(outSize);

		auto memInfo=Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
		inTensor=Ort::Value::CreateTensor<float>(memInfo, omp.in.data(), inSize, inShape.data(), 2);
		outTensor=Ort::Value::CreateTensor(memInfo, omp.out.data(), outSize, outShape.data(), 2);

		inNamePtr=inName.data();
		outNamePtr=outName.data();
		inPtr=inTensor.GetTensorMutableData<float>();
		outPtr=outTensor.GetTensorMutableData<float>();
		return 1;
	}
	void onnxClass::impClass::run(){
		if(omp.in.size()!=inSize){
			stringstream ss;
			ss<<"onnx输入size不匹配！onnx要求size="<<inSize<<"，调用传入in.size="<<omp.in.size();
			throw runtime_error(ss.str());
		}
		memcpy(inPtr, omp.in.data(), inSize*4);
		ort->Run(runOpts, &inNamePtr, &inTensor, 1, &outNamePtr, &outTensor, 1);
		memcpy(omp.out.data(), outPtr, outSize*4);
	}
// =======================================
	onnxClass::onnxClass():imp(*new impClass(this)){}
	bool onnxClass::init(const string& onnxFile, const string &name){
		return imp.init(onnxFile, name);
	}
	void onnxClass::run(){
		imp.run();
	}
	const string &onnxClass::getName(){
		return imp.name;
	}
	onnxClass::~onnxClass(){
		if(imp.ort){delete imp.ort;}
		delete &imp;
	}
}//namespace
