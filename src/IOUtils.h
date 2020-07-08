//
// IOUtils.h
// Utilities dealing with file i/o
//
// Created by: Jeremy Keller
// 2/06/2020
//

#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <unordered_map>
#include "BodyTrackingUtils.h"

#ifndef IO_UTILS_H
#define IO_UTILS_H

/**
* Utilities dealing with file i/o, mostly focusing on reading and parsing in files
*/
class IOUtils
{
private:
	/**
	* Reads a line of a 4x4 transformation line and converts it to a 4 element Eigen RowVector
	*
	* @param line Text line of 4 elements of a transformation separated by whitespace
	* @return Parsed transform line
	*/
	static Eigen::RowVector4d readTransformLine(std::string line);

	/**
	* Populates the min/max bounds of a BoundingBox based on a text line with the coordinate axis and min/max allowed values
	*
	* @param bounds BoundingBox to populate
	* @param line Text line to parse into BoundingBox parameters. In the form of "{Axis (x/y/z)} {minValue} {maxValue}" 
	*/
	static void populateBounds(BodyTrackingUtils::BoundingBox* bounds, std::string line);

public:
	/**
	* Check if a string ends with the specified suffix
	*
	* @param fullString String to check against
	* @param ending Suffix to check
	* @return Whether or not the given string ends in the specified suffix
	*/
	static bool endsWith(std::string const fullString, std::string const ending);

	/**
	* Extracts the file name from a full file path
	* 
	* @param fileName File name which can include the full directory path to it
	* @return The isolated file name without a file extension suffix
	*/
	static std::string IOUtils::extractFileName(std::string fileName);

	/**
	* Obtains Kinect .mkv captures from the given directory and stores their file paths
	* 
	* @param dirPath Directory path to search for .mkv files in
	* @return Vector of file names corresponding to the .mkv files in the specified path
	*/
	static std::vector<std::string> obtainMkvFilesFromDirectory(std::string dirPath);

	/**
	* Reads a transformation file and populates 4x4 matrix transformations corresponding to file suffixes
	*
	* @param fileName Full file name with path of the transform file to process
	* @return Map of file ending suffixes to 4x4 transformation matrices
	*/
	static std::unordered_map<std::string, Eigen::Matrix4Xd> readTransformationFile(std::string fileName);

	/**
	* Obtains the body tracking file suffix from a transformation file
	*
	* @param fileName Full file name with path of the transform file to process
	* @param skipBtCloudProcessing Pointer to boolean which will be set if the body tracking capture should not be processed into point cloud and only used for its joint location tracking.
	*	This can be configured in the configuration file by adding a dash '-' to the end of the body tracking file suffix symbol.
	*	It is required when running the body tracking capture at a higher frame rate than the rest of the captures.
	* @return The file suffix to use for body tracking. Empty string if not specified in the transform file
	*/
	static std::string obtainBodyTrackingFileSuffix(std::string fileName, bool* skipBtCloudProcessing);

	/**
	* Obtains capture space bounds from a transformation file
	*
	* @param fileName Full file name with path of the transform file to process
	* @return BoundingBox of the capture space to process. 
	*	An unbound capture space will be created if no capture space is specified in the transform file.
	*/
	static BodyTrackingUtils::BoundingBox obtainCaptureSpaceBounds(std::string fileName);
};
#endif 

