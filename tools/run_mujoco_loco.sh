#!/bin/bash

echo run_mujoco_loco.sh 将删除最近10个以外的log文件
cd ../../loong_sim/log
files=$(ls -lt | tail -n +11 | awk '{print $9}')
for file in $files; do
	rm -f "$file"
done
echo ==========

cd ../build
./main_loco $1
