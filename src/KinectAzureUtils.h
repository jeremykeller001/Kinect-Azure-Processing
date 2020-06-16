//
// KinectAzureUtils.h
// Custom Utilities dealing with the Kinect Azure SDK
//
// Created by: Jeremy Keller
// 2/03/2020
//


#pragma once
#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <k4abt.h>
#include <math.h>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <stdio.h>
#include <malloc.h>
#include <vector>
#include "BodyTrackingUtils.h"
#include "IOUtils.h"
#include "MatrixUtils.h"
#include "Ply.h"

#ifndef KINECT_AZURE_UTILS_H
#define KINECT_AZURE_UTILS_H

class KinectAzureUtils
{
private:
	std::string captureDirectory;
	int individualFrameIndex;
	bool calibrationMode;
	bool debugMode;
	bool disableMeshOutput;
	bool bodyTrackingOutputOnly;

	static const uint64_t timestampDiff15 = 66000;
	static const uint64_t timestampDiff30 = 33000;

	typedef struct
	{
		std::string fileName;
		uint64_t timestampDiff;
	} CalibrationInfo;

	typedef struct
	{
		std::string fileName;
		int timestampCount;
	} TimestampCounter;

	typedef struct
	{
		std::string filename;
		k4a_playback_t handle;
		k4a_record_configuration_t record_config;
		k4a_capture_t capture;
		int index;
	} recording_t;

	typedef struct
	{
		std::string fileName;
		int fileCount;
		int* indexCounts = new int[fileCount - 1];
	} FileIndexCounter;

public:
	typedef struct
	{
		int index;
		uint64_t timestamp;
		KinectAzureUtils::recording_t* file;
	} FrameInfo;

private:
	std::string getStartRecording(recording_t* files, std::vector<CalibrationInfo> frameInfo, int fileCount);

	void createXYTable(const k4a_calibration_t* calibration, k4a_image_t xy_table);

	// For debugging
	void print_capture_info(recording_t* file);

	uint64_t first_capture_timestamp(k4a_capture_t capture);

	FrameInfo getNextFrame(int fileCount, recording_t* files);

	Ply generatePly(FrameInfo frameInfo, const k4a_image_t xyTable);

	Ply generatePointCloud(FrameInfo frameInfo, k4a_calibration_t* calibrations);

	bool checkSubjectWithinCaptureSpace(std::vector<Eigen::RowVector3d> jointPositions, BodyTrackingUtils::BoundingBox captureSpaceBounds);

	void outputPointCloudGroup(std::vector<Ply> plys, uint64_t groupCount, std::unordered_map<std::string, Eigen::Matrix4Xd> transformations, 
		std::vector<Eigen::RowVector3d> jointPositions, BodyTrackingUtils::BoundingBox captureSpaceBounds, std::string bodyTrackingFileSuffix);

	bool openFiles(KinectAzureUtils::recording_t** files, k4a_calibration_t** calibrations, k4abt_tracker_t& tracker, std::vector<std::string> mkvFiles, std::string btFileSuffix);

public:
	KinectAzureUtils(std::string captureDir) {
		captureDirectory = captureDir;
	}

	void setIndividualFrameIndex(int frameIndex) {
		individualFrameIndex = frameIndex;
	}

	void setCalibrationMode(bool cal) {
		calibrationMode = cal;
	}

	void setDebugMode(bool debug) {
		debugMode = debug;
	}

	void setDisableMeshOutput(bool disableMesh) {
		disableMeshOutput = disableMesh;
	}

	void setBodyTrackingOutputOnly(bool bodyTrackingOnly) {
		bodyTrackingOutputOnly = bodyTrackingOnly;
	}

	int outputRecordingsToPlyFiles(std::unordered_map<std::string, Eigen::Matrix4Xd> transformations, std::string btFileSuffix, BodyTrackingUtils::BoundingBox captureSpaceBounds);
};
#endif
