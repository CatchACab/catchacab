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
* \brief The entry point for the Inference Engine interactive_face_detection demo application
* \file interactive_face_detection_demo/main.cpp
* \example interactive_face_detection_demo/main.cpp
*/
#include <gflags/gflags.h>
#include <functional>
#include <iostream>
#include <fstream>
#include <random>
#include <memory>
#include <chrono>
#include <vector>
#include <string>
#include <utility>
#include <algorithm>
#include <iterator>
#include <map>

#include <inference_engine.hpp>

#include <samples/common.hpp>
#include <samples/slog.hpp>

#include "interactive_face_detection.hpp"
#include "detectors.hpp"

#include <ie_iextension.h>
#include <ext_list.hpp>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <fstream>

using namespace InferenceEngine;


bool ParseAndCheckCommandLine(int argc, char *argv[]) {
	// ---------------------------Parsing and validation of input args--------------------------------------
	gflags::ParseCommandLineNonHelpFlags(&argc, &argv, true);
	if (FLAGS_h) {
		showUsage();
		return false;
	}
	slog::info << "Parsing input parameters" << slog::endl;

	if (FLAGS_i.empty()) {
		throw std::logic_error("Parameter -i is not set");
	}

	if (FLAGS_m.empty()) {
		throw std::logic_error("Parameter -m is not set");
	}

	if (FLAGS_n_ag < 1) {
		throw std::logic_error("Parameter -n_ag cannot be 0");
	}

	if (FLAGS_n_hp < 1) {
		throw std::logic_error("Parameter -n_hp cannot be 0");
	}

	// no need to wait for a key press from a user if an output image/video file is not shown.
	FLAGS_no_wait |= FLAGS_no_show;

	return true;
}


int main(int argc, char *argv[]) {

	float prop_facing_car = 0.f;
	float prop_happy = 0.f;



	//hyperparameter
	float reset_time = 3500.f;
	float yawradian = 20.0f;

	unsigned int happy_count = 0;
	unsigned int emotion_count = 0;


	float reset_duration = 0.f;
	float max_time_watched = 0.f;
	float current_time_watched = 0.f;


	try {
		std::cout << "InferenceEngine: " << GetInferenceEngineVersion() << std::endl;

		// ------------------------------ Parsing and validation of input args ---------------------------------
		if (!ParseAndCheckCommandLine(argc, argv)) {
			return 0;
		}

		slog::info << "Reading input" << slog::endl;
		cv::VideoCapture cap;
		const bool isCamera = FLAGS_i == "cam";
		if (!(FLAGS_i == "cam" ? cap.open(0) : cap.open(FLAGS_i))) {
			throw std::logic_error("Cannot open input file or camera: " + FLAGS_i);
		}
		const size_t width = (size_t)cap.get(cv::CAP_PROP_FRAME_WIDTH);
		const size_t height = (size_t)cap.get(cv::CAP_PROP_FRAME_HEIGHT);

		// read input (video) frame
		cv::Mat frame;
		if (!cap.read(frame)) {
			throw std::logic_error("Failed to get frame from cv::VideoCapture");
		}
		// -----------------------------------------------------------------------------------------------------
		// --------------------------- 1. Load Plugin for inference engine -------------------------------------
		std::map<std::string, InferencePlugin> pluginsForDevices;
		std::vector<std::pair<std::string, std::string>> cmdOptions = {
			{ FLAGS_d, FLAGS_m },{ FLAGS_d_ag, FLAGS_m_ag },{ FLAGS_d_hp, FLAGS_m_hp },
		{ FLAGS_d_em, FLAGS_m_em },{ FLAGS_d_lm, FLAGS_m_lm }
		};
		FaceDetection faceDetector(FLAGS_m, FLAGS_d, 1, false, FLAGS_async, FLAGS_t, FLAGS_r);
		AgeGenderDetection ageGenderDetector(FLAGS_m_ag, FLAGS_d_ag, FLAGS_n_ag, FLAGS_dyn_ag, FLAGS_async);
		HeadPoseDetection headPoseDetector(FLAGS_m_hp, FLAGS_d_hp, FLAGS_n_hp, FLAGS_dyn_hp, FLAGS_async);
		EmotionsDetection emotionsDetector(FLAGS_m_em, FLAGS_d_em, FLAGS_n_em, FLAGS_dyn_em, FLAGS_async);
		FacialLandmarksDetection facialLandmarksDetector(FLAGS_m_lm, FLAGS_d_lm, FLAGS_n_lm, FLAGS_dyn_lm, FLAGS_async);

		for (auto && option : cmdOptions) {
			auto deviceName = option.first;
			auto networkName = option.second;

			if (deviceName == "" || networkName == "") {
				continue;
			}

			if (pluginsForDevices.find(deviceName) != pluginsForDevices.end()) {
				continue;
			}
			slog::info << "Loading plugin " << deviceName << slog::endl;
			InferencePlugin plugin = PluginDispatcher({ "../../../lib/intel64", "" }).getPluginByDevice(deviceName);

			/** Printing plugin version **/
			printPluginVersion(plugin, std::cout);

			/** Load extensions for the CPU plugin **/
			if ((deviceName.find("CPU") != std::string::npos)) {
				plugin.AddExtension(std::make_shared<Extensions::Cpu::CpuExtensions>());

				if (!FLAGS_l.empty()) {
					// CPU(MKLDNN) extensions are loaded as a shared library and passed as a pointer to base extension
					auto extension_ptr = make_so_pointer<IExtension>(FLAGS_l);
					plugin.AddExtension(extension_ptr);
					slog::info << "CPU Extension loaded: " << FLAGS_l << slog::endl;
				}
			}
			else if (!FLAGS_c.empty()) {
				// Load Extensions for other plugins not CPU
				plugin.SetConfig({ { PluginConfigParams::KEY_CONFIG_FILE, FLAGS_c } });
			}
			pluginsForDevices[deviceName] = plugin;
		}

		/** Per layer metrics **/
		if (FLAGS_pc) {
			for (auto && plugin : pluginsForDevices) {
				plugin.second.SetConfig({ { PluginConfigParams::KEY_PERF_COUNT, PluginConfigParams::YES } });
			}
		}
		// -----------------------------------------------------------------------------------------------------

		// --------------------------- 2. Read IR models and load them to plugins ------------------------------
		// Disable dynamic batching for face detector as long it processes one image at a time.
		Load(faceDetector).into(pluginsForDevices[FLAGS_d], false);
		Load(ageGenderDetector).into(pluginsForDevices[FLAGS_d_ag], FLAGS_dyn_ag);
		Load(headPoseDetector).into(pluginsForDevices[FLAGS_d_hp], FLAGS_dyn_hp);
		Load(emotionsDetector).into(pluginsForDevices[FLAGS_d_em], FLAGS_dyn_em);
		Load(facialLandmarksDetector).into(pluginsForDevices[FLAGS_d_lm], FLAGS_dyn_lm);
		// -----------------------------------------------------------------------------------------------------

		// --------------------------- 3. Do inference ---------------------------------------------------------
		// Start inference & calc performance.

		bool isFaceAnalyticsEnabled = ageGenderDetector.enabled() || headPoseDetector.enabled() ||
			emotionsDetector.enabled() || facialLandmarksDetector.enabled();

		Timer timer;

		std::ostringstream out;
		size_t framesCounter = 0;
		bool frameReadStatus;
		bool isLastFrame;
		cv::Mat prev_frame, next_frame;

		// Detect all faces on the first frame and read the next one.

		faceDetector.enqueue(frame);
		faceDetector.submitRequest();


		prev_frame = frame.clone();

		// Read next frame.
		frameReadStatus = cap.read(frame);

		std::fstream file;
		file.open("facial", std::fstream::in | std::fstream::out | std::fstream::app);
		file << "]start" << std::endl;
		while (true) {
			timer.start("reset");
			isLastFrame = !frameReadStatus;


			// Retrieve face detection results for previous frame.
			faceDetector.wait();
			faceDetector.fetchResults();
			auto prev_detection_results = faceDetector.results;

			// No valid frame to infer if previous frame is last.
			if (!isLastFrame) {
				faceDetector.enqueue(frame);
				faceDetector.submitRequest();
			}



			// Fill inputs of face analytics networks.
			for (auto &&face : prev_detection_results) {
				if (isFaceAnalyticsEnabled) {
					auto clippedRect = face.location & cv::Rect(0, 0, width, height);
					cv::Mat face = prev_frame(clippedRect);
					ageGenderDetector.enqueue(face);
					headPoseDetector.enqueue(face);
					emotionsDetector.enqueue(face);
					facialLandmarksDetector.enqueue(face);
				}
			}


			// Run age-gender recognition, head pose estimation and emotions recognition simultaneously.

			if (isFaceAnalyticsEnabled) {
				ageGenderDetector.submitRequest();
				headPoseDetector.submitRequest();
				emotionsDetector.submitRequest();
				facialLandmarksDetector.submitRequest();
			}


			// Read next frame if current one is not last.
			if (!isLastFrame) {

				frameReadStatus = cap.read(next_frame);

			}

			if (isFaceAnalyticsEnabled) {
				ageGenderDetector.wait();
				headPoseDetector.wait();
				emotionsDetector.wait();
				facialLandmarksDetector.wait();
			}




			// For every detected face.

			auto &result = prev_detection_results[0];
			cv::Rect rect = result.location;

			out.str("");

			if (ageGenderDetector.enabled() && 0 < ageGenderDetector.maxBatch) {
				out << (ageGenderDetector[0].maleProb > 0.5 ? "M" : "F");
				out << std::fixed << std::setprecision(0) << "," << ageGenderDetector[0].age;
				if (FLAGS_r) {
					std::cout << "Predicted gender, age = " << out.str() << std::endl;
				}
			}
			else {
				out << (result.label < faceDetector.labels.size() ? faceDetector.labels[result.label] :
					std::string("label #") + std::to_string(result.label))
					<< ": " << std::fixed << std::setprecision(3) << result.confidence;
			}

			if (emotionsDetector.enabled() && 0 < emotionsDetector.maxBatch) {
				std::string emotion = emotionsDetector[0];
				std::cout << emotion << std::endl;

				if (emotion == "happy" || emotion == "neutral") { happy_count = happy_count + 1; }
				emotion_count = emotion_count + 1;
			}


			if (headPoseDetector.enabled() && 0 < headPoseDetector.maxBatch) {
				if (true) {
					//std::cout << "Head pose results: yaw, pitch, roll = "
					//        << headPoseDetector[i].angle_y << ";"
					//      << headPoseDetector[i].angle_p << ";"
					//    << headPoseDetector[i].angle_r << std::endl;

					if (headPoseDetector[0].angle_y <= yawradian && headPoseDetector[0].angle_y >= (-1.0f*yawradian))
					{
						timer.finish("reset"); current_time_watched = current_time_watched + timer["reset"].getSmoothedDuration(); timer.start("watch");
						if (current_time_watched> max_time_watched) { max_time_watched = current_time_watched; }
					}
					else { timer.finish("reset"); current_time_watched = 0.f; }
					//std::cout << "currentTime: " << current_time_watched << "max time_watched: " << max_time_watched << std::endl;
				}

			}


			timer.finish("reset");
			reset_duration = reset_duration + timer["reset"].getSmoothedDuration();
			if (reset_duration> reset_time) {
				//reset timers and values
				reset_duration = 0.f;
				max_time_watched = 0.f;
				current_time_watched = 0.f;

				happy_count = 0;
				emotion_count = 1; // to avoide division by zero
				std::cout << "reset: " << timer["reset"].getSmoothedDuration() << std::endl;



			}


			//propabilities

			prop_facing_car = (max_time_watched / reset_duration);
			prop_happy = ((1.0f*happy_count) / emotion_count);

			std::cout << "facingcar: " << prop_facing_car << "happy: " << prop_happy << std::endl;
			file << "]" << prop_facing_car << ":" << prop_happy << std::endl;
			// End of file (or a single frame file like an image). We just keep last frame displayed to let user check what was shown
			if (isLastFrame) {

				if (!FLAGS_no_wait) {
					std::cout << "No more frames to process. Press any key to exit" << std::endl;
					cv::waitKey(0);
				}
				break;
			}
			else if (!FLAGS_no_show && -1 != cv::waitKey(1)) {

				break;
			}

			prev_frame = frame;
			frame = next_frame;
			next_frame = cv::Mat();
		}

		file << "]end" << std::endl;


		// -----------------------------------------------------------------------------------------------------
	}
	catch (const std::exception& error) {
		slog::err << error.what() << slog::endl;
		return 1;
	}
	catch (...) {
		slog::err << "Unknown/internal exception happened." << slog::endl;
		return 1;
	}

	slog::info << "Execution successful" << slog::endl;
	return 0;
}


