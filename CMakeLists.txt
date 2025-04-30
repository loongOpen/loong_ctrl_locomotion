#[[
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

=====================================================]]

cmake_minimum_required(VERSION 3.1)
project(nabo)

option(cpu_x64 "cpu x64" 1)
option(sim_mode "sim mode" 1)
option(rl_use_cuda "rl use cuda" 1)

if(sim_mode)
	add_definitions(-DDefSim)
endif()
if(rl_use_cuda)
	add_definitions(-DDefCuda)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -rdynamic")
# set(CMAKE_BUILD_WITH_INSTALL_RPATH 1)
# set_property(GLOBAL NO_SONAME 1)
# set(NO_SONAME 1)

if(UNIX)
	add_definitions(-std=c++17)
	add_compile_options(-fPIC)
	if(NOT cpu_x64)
		set(CMAKE_C_COMPILER "/usr/bin/aarch64-linux-gnu-gcc-9")
		set(CMAKE_CXX_COMPILER "/usr/bin/aarch64-linux-gnu-g++-9")
	endif()
else()
	# add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
	add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
	set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS on)
	set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()
# ================================
include(../loong_utility/loong_utility.cmake)

# 第三方.h
set(incPreFix ${CMAKE_SOURCE_DIR}/third_party/include)
file(GLOB children RELATIVE ${incPreFix} ${incPreFix}/*)
foreach(child ${children})
	if(IS_DIRECTORY ${incPreFix}/${child})
		list(APPEND allInc ${incPreFix}/${child})
	endif()
endforeach()
# 项目.h
set(srcPreFix ${CMAKE_SOURCE_DIR}/src)
file(GLOB children RELATIVE ${srcPreFix} ${srcPreFix}/*)
foreach(child ${children})
	if(IS_DIRECTORY ${srcPreFix}/${child})
		list(APPEND allInc ${srcPreFix}/${child})
	endif()
endforeach()

include_directories(${allInc})
# ================================
if(${CMAKE_CXX_COMPILER} MATCHES "aarch64" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
	message("=== linux arm64架构 ===")
	link_directories("third_party/lib_lin_a64")
	link_directories("third_party/lib_lin_a64/onnxruntime")
	set(CMAKE_INSTALL_RPATH "$ORIGIN;$ORIGIN/../../third_party/lib_lin_a64;")
	set(nabo nabo_a64)
else()
	message("=== linux x64架构 ===")
	link_directories("third_party/lib_lin_x64")
	link_directories("third_party/lib_lin_x64/onnxruntime")
	set(CMAKE_INSTALL_RPATH "$ORIGIN;$ORIGIN/../../third_party/lib_lin_x64;")
	set(nabo nabo_x64)
endif()
# ================================
# -----------
file(GLOB_RECURSE srcFiles "src/*.cpp")
add_library(${nabo} SHARED ${srcFiles})
set_target_properties(${nabo} PROPERTIES NO_SONAME 1)

set(coreLibs util onnxruntime)

if(MSVC)
	target_link_libraries(${nabo} util)
elseif(UNIX)
	target_link_libraries(${nabo} ${coreLibs} pthread)
endif()
# -----------
aux_source_directory(test testFiles)
add_executable(main test/main.cpp)
target_link_libraries(main ${nabo})
# ===============================
set(CMAKE_INSTALL_PREFIX ../nabo_output)
install(TARGETS ${nabo} LIBRARY DESTINATION .)
# install(FILES "src/manager/nabo.h" "src/manager/nabo_data.h" DESTINATION .)