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

#ifndef KINECT_AZURE_PROCESSOR_H
#define KINECT_AZURE_PROCESSOR_H

/**
* Class for processing Kinect Azure captures
*/
class KinectAzureProcessor
{
private:
	/**
	* Directory of the Kinect captures to process
	*/
	std::string captureDirectory;

	/**
	* Specifies the frame group number if only an individual frame is set to be output. 
	* Default value is set to -1 otherwise.
	*/
	int individualFrameIndex;

	/**
	* Specifies whether this processor will run in calibration mode or not. 
	* Calibration mode will output a single group frame point cloud for external calibration integrations use.
	*/
	bool calibrationMode;

	/**
	* Specifies whether this processor will run in debug mode or not. 
	* When running in debug mode, additional file outputs are generated to aid any debugging efforts.
	*/
	bool debugMode;

	/**
	* Specifies whether or not mesh outputs will be disabled. 
	* Disabling mesh outputs will greatly speed up processing time.
	*/
	bool disableMeshOutput;

	/**
	* Specifies whether only body tracking output will be generated. 
	* If set to true, no point clouds will be output, only the joints.json file for the entire capture.
	*/
	bool bodyTrackingOutputOnly;

	/**
	* Specifies if cloud processing will be skipped on the body tracking camera.
	* This is required for cases where the body tracking camera is run at a higher frame rate than the others.
	* When specified, only the joint locations will be used from the body tracking capture.
	*/
	bool skipBtCloudProcessing;

	/**
	* Timestamp difference between frames when running at 15 frames per second
	*/
	static const uint64_t timestampDiff15 = 66000;

	/**
	* Timestamp difference between frames when running at 30 frames per second
	*/
	static const uint64_t timestampDiff30 = 33000;

	/**
	* Calibration information holder
	*/
	typedef struct
	{
		std::string fileName;
		uint64_t timestampDiff;
	} CalibrationInfo;

	/**
	* Holds a count of timestamp differences between a current file's frame and the most recent frame before it
	*/
	typedef struct
	{
		std::string fileName;
		int timestampCount;
	} TimestampCounter;

	/**
	* Holds recording information for a single Kinect camera
	*/
	typedef struct
	{
		std::string filename;
		k4a_playback_t handle;
		k4a_record_configuration_t record_config;
		k4a_capture_t capture;
		int index;
	} recording_t;

	/**
	* Holds information to track the occurences of a frames coming from specific file
	*/
	typedef struct
	{
		std::string fileName;
		int fileCount;
		int* indexCounts = new int[fileCount - 1];
	} FileIndexCounter;

public:

	/**
	* Holds information for a single frame to be processed
	*/
	typedef struct
	{
		int index;
		uint64_t timestamp;
		KinectAzureProcessor::recording_t* file;
	} FrameInfo;

private:

	/**
	* Gets the name of the recording to start the first output group with
	*
	* @param files Files to process
	* @param frameInfo Calibration info tracked from previous frames read in
	* @param fileCount Number of files to process
	* @return Name of the file to start the first output group with 
	*/
	std::string getStartRecording(recording_t* files, std::vector<CalibrationInfo> frameInfo, int fileCount);

	/**
	* Prints capture info with timestamp for a frame.
	* Useful for debugging
	*
	* @param file File to print capture info for
	*/
	void print_capture_info(recording_t* file);

	/**
	* Obtains the timestamp of the depth image contained in a capture
	*
	* @param capture Capture to process
	* @return Timestamp of the depth image contained within the capture
	*/
	uint64_t first_capture_timestamp(k4a_capture_t capture);

	/**
	* Gets the next depth frame to process from a group of capture files
	* 
	* @param fileCount Number of files being processed
	* @param files Container holding the Kinect capture files being processed
	* @return Frame information of the next frame to process chronologically
	*/
	FrameInfo getNextFrame(int fileCount, recording_t* files);

	/**
	* Generates a point cloud from depth data contained within a frame
	*
	* @param frameInfo Frame data to process
	* @param calibrations Calibration info for the current set of captures
	* @return Generated point cloud
	*/
	Ply generatePointCloud(FrameInfo frameInfo, k4a_calibration_t* calibrations);

	/**
	* Checks if the subject is within the designated capture space based on calculated joint positions
	*
	* @param jointPositions Joint positions of the subject
	* @param captureSpaceBounds Bounds of the capture space, as determined in the transform file
	* @return Whether or not the subject is within the capture space. If any joint is outside of the capture space, it returns false.
	*/
	bool checkSubjectWithinCaptureSpace(std::vector<Eigen::RowVector3d> jointPositions, BodyTrackingUtils::BoundingBox captureSpaceBounds);

	void outputPointCloudGroup(std::vector<Ply> plys, uint64_t groupCount, std::unordered_map<std::string, Eigen::Matrix4Xd> transformations, 
		std::vector<Eigen::RowVector3d> jointPositions, BodyTrackingUtils::BoundingBox captureSpaceBounds, std::string bodyTrackingFileSuffix);

	bool openFiles(KinectAzureProcessor::recording_t** files, k4a_calibration_t** calibrations, k4abt_tracker_t& tracker, std::vector<std::string> mkvFiles, std::string btFileSuffix);

public:
	KinectAzureProcessor(std::string captureDir) {
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

	void setSkipBtCloudProcessing(bool skipBtCloud) {
		skipBtCloudProcessing = skipBtCloud;
	}

	int outputRecordingsToPlyFiles(std::unordered_map<std::string, Eigen::Matrix4Xd> transformations, std::string btFileSuffix, BodyTrackingUtils::BoundingBox captureSpaceBounds);
};
#endif
