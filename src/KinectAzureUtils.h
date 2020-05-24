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
#include "IOUtils.h"
#include "Ply.h"
#include "IOUtils.h"
#include "MatrixUtils.h"

#ifndef KINECT_AZURE_UTILS_H
#define KINECT_AZURE_UTILS_H

class KinectAzureUtils
{
private:
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
	static std::string getStartRecording(recording_t* files, std::vector<CalibrationInfo> frameInfo, int fileCount);

	static void createXYTable(const k4a_calibration_t* calibration, k4a_image_t xy_table);

	// For debugging
	static void print_capture_info(recording_t* file);

	static uint64_t first_capture_timestamp(k4a_capture_t capture);

	static FrameInfo getNextFrame(int fileCount, recording_t* files);

	static Ply generatePly(FrameInfo frameInfo, const k4a_image_t xyTable);

	static Ply generatePointCloud(FrameInfo frameInfo, k4a_calibration_t* calibrations);

	static void outputPointCloudGroup(std::vector<Ply> plys, uint64_t groupCount, std::unordered_map<std::string, Eigen::Matrix4Xd> transformations, 
		std::string outputPath, std::vector<Eigen::RowVector3d> jointPositions, std::string bodyTrackingFileSuffix, bool calibrationMode, bool debugMode, bool skipMesh);

	static bool openFiles(KinectAzureUtils::recording_t** files, k4a_calibration_t** calibrations, k4abt_tracker_t& tracker, std::vector<std::string> mkvFiles, std::string btFileSuffix);

public:
	static int outputRecordingsToPlyFiles(std::string dirPath, std::string transformPath, int frameOutputNumber, bool calibrationMode, bool debugMode, bool skipMesh);
};
#endif
