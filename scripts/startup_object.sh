#!/bin/bash

cp /home/rho2/dev/hackatum/catchacab/res/object_detection_demo_ssd_async/main.cpp /opt/intel/computer_vision_sdk_2018.4.420/deployment_tools/inference_engine/samples/object_detection_demo_ssd_async

cd /home/rho2/dev/hackatum/Hackatum/
source /opt/intel/computer_vision_sdk_2018.4.420/bin/setupvars.sh
echo "Sourced the stuff"
make 

/home/rho2/dev/hackatum/Hackatum/intel64/Release/./object_detection_demo_ssd_async -i cam -m /home/rho2/dev/hackatum/catchacab/res/object_detection_demo_ssd_async/frozen_inference_graph.xml