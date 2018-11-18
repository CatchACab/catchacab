/*
// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

/**
* \brief The entry point for the Inference Engine Human Pose Estimation demo application
* \file human_pose_estimation_demo/main.cpp
* \example human_pose_estimation_demo/main.cpp
*/

#include <vector>
#include <stdio.h>

#include <inference_engine.hpp>
#include <opencv2/opencv.hpp>

#include <samples/common.hpp>

#include "human_pose_estimation_demo.hpp"
#include "human_pose_estimator.hpp"
#include "render_human_pose.hpp"

using namespace InferenceEngine;
using namespace human_pose_estimation;

float x_val;

bool ParseAndCheckCommandLine(int argc, char* argv[]) {
    // ---------------------------Parsing and validation of input args--------------------------------------

    gflags::ParseCommandLineNonHelpFlags(&argc, &argv, true);
    if (FLAGS_h) {
        showUsage();
        return false;
    }

    std::cout << "[ INFO ] Parsing input parameters" << std::endl;

    if (FLAGS_i.empty()) {
        throw std::logic_error("Parameter -i is not set");
    }

    if (FLAGS_m.empty()) {
        throw std::logic_error("Parameter -m is not set");
    }

    return true;
}

int main(int argc, char* argv[]) {
int number = 4;
int offset = 120;
int size = 300;
    try {
        std::cout << "InferenceEngine: " << GetInferenceEngineVersion() << std::endl;
	
        // ------------------------------ Parsing and validation of input args ---------------------------------
        if (!ParseAndCheckCommandLine(argc, argv)) {
            return EXIT_SUCCESS;
        }

        HumanPoseEstimator estimator(FLAGS_m, FLAGS_d, FLAGS_pc);
        cv::VideoCapture cap;
        if (!(FLAGS_i == "cam" ? cap.open(0) : cap.open(FLAGS_i))) {
            throw std::logic_error("Cannot open input file or camera: " + FLAGS_i);
        }

	

        int delay = 33;
        double inferenceTime = 0.0;
        cv::Mat image;
        while (cap.read(image)) {
int x_val =0;
	int y_val = 0;
            
          
            std::vector<HumanPose> poses = estimator.estimate(image);
           int pose_score = 0;
     
                for (HumanPose const& pose : poses) {
   if(pose.score >=pose_score){
pose_score = pose.score;

// get size of original image
cv::Size s = image.size();
if(pose.keypoints[number].x >= 0 && pose.keypoints[number].y >= 0 && pose.keypoints[number].x <= s.width-(size/2) && pose.keypoints[number].y <= s.height-(size/2)){
		x_val= pose.keypoints[number].x-(size/2);
if(x_val <= 0 ){
x_val = 0;
}
y_val= pose.keypoints[number].y-(size/2)-offset;
if(y_val <= 0 ){
y_val = 0;
}
}
}
                   



                }
           

// Setup a rectangle to define your region of interest
cv::Rect myROI(x_val, y_val, size, size);

// Crop the full image to that image contained by the rectangle myROI
// Note that this doesn't copy the data
cv::Mat croppedImage = image(myROI);

            cv::imshow("ICV Human Pose Estimation", croppedImage);

			renderHumanPose(poses, image);

            int key = cv::waitKey(delay) & 255;
            if (key == 'p') {
                delay = (delay == 0) ? 33 : 0;
            } else if (key == 27) {
                break;
            }
        }
    }
    catch (const std::exception& error) {
        std::cerr << "[ ERROR ] " << error.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (...) {
        std::cerr << "[ ERROR ] Unknown/internal exception happened." << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "[ INFO ] Execution successful" << std::endl;
    return EXIT_SUCCESS;
}
