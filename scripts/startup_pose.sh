#!/bin/bash

#cp res/object_detection_demo_ssd_async/main.cpp /opt/intel/computer_vision_sdk_2018.4.420/deployment_tools/inference_engine/samples/object_detection_demo_ssd_async/

cp  /home/rho2/dev/hackatum/catchacab/res/human_pose_estimation_demo/src/render_human_pose.cpp \
    /opt/intel/computer_vision_sdk_2018.4.420/deployment_tools/inference_engine/samples/human_pose_estimation_demo/src
cp  /home/rho2/dev/hackatum/catchacab/res/human_pose_estimation_demo/main.cpp \
    /opt/intel/computer_vision_sdk_2018.4.420/deployment_tools/inference_engine/samples/human_pose_estimation_demo

cd /home/rho2/dev/hackatum/Hackatum/
source /opt/intel/computer_vision_sdk_2018.4.420/bin/setupvars.sh
echo "Sourced the stuff"
make

/home/rho2/dev/hackatum/Hackatum/intel64/Release/./human_pose_estimation_demo -i cam -m /opt/intel/computer_vision_sdk_2018.4.420/deployment_tools/intel_models/human-pose-estimation-0001/FP32/human-pose-estimation-0001.xml 

