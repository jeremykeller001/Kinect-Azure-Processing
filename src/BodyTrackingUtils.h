//
// BodyTrackingUtils.h
// Custom Utilities dealing with the Kinect Body Tracking SDK
// Adopted from code originally written by Eddy.Rogers@du.edu
//
// Created by: Jeremy Keller
// 3/16/2020
//

#pragma once

#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <Eigen/Dense>

#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <k4abt.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "BodyTrackingHelpers.h"
#include "KinectAzureUtils.h"
#include "MatrixUtils.h"

#ifndef BODY_TRACKING_UTILS_H
#define BODY_TRACKING_UTILS_H

class BodyTrackingUtils
{
public:
	// Eddy Rogers
	//Struct to contain the minimum and maximum acceptable x, y, and z values
	struct BoundingBox {
		double xMin;
		double xMax;
		double yMin;
		double yMax;
		double zMin;
		double zMax;
	};

	static bool predictJoints(boost::property_tree::ptree* jsonFrames, int frameCount, k4abt_tracker_t tracker, k4a_capture_t capture_handle, std::vector<Eigen::RowVector3d>* jointPositions);

	//static void outputJoints(int frameCount, k4a_float3_t jointPositions);

	static BoundingBox createBoundingBox(std::vector<Eigen::RowVector3d> jointPositions);

	static bool withinBounds(Eigen::RowVector3d point, BoundingBox bounds);
	
};
#endif