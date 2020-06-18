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

#ifndef BODY_TRACKING_UTILS_H
#define BODY_TRACKING_UTILS_H

/**
* Utilities to obtain joint locations from body tracking data and perform calculations based on them
*/
class BodyTrackingUtils
{
public:
	/**
	* Struct to contain the minimum and maximum acceptable x, y, and z values for a point cloud.
	* It is intended to be used to filter out points outside of the relevant capture space.
	*/
	struct BoundingBox {
		double xMin;
		double xMax;
		double yMin;
		double yMax;
		double zMin;
		double zMax;
	};

	/**
	* Predict the joints of a frame and output the locations to a json object
	*
	* @param jsonFrames Json object to append the joint locations of the frame to
	* @param frameCount The frame group number of the joint locations being predicted
	* @param tracker Body tracker
	* @param capture_handle Capture to process
	* @param jointPositions Container that will hold the predicted joint locations to be used for later processing
	* @return Whether or not joints could be predicted for the capture frame
	*/
	static bool predictJoints(boost::property_tree::ptree* jsonFrames, int frameCount, k4abt_tracker_t tracker, k4a_capture_t capture_handle, std::vector<Eigen::RowVector3d>* jointPositions);

	/**
	* Generates a BoundingBox based on joint locations so that a subject may be isolated within a capture
	*
	* @param jointPositions Joint positions to use to generate the BoundingBox with
	* @return BoundingBox object with xyz min/max values populated based on the joint positions
	*/
	static BoundingBox createBoundingBox(std::vector<Eigen::RowVector3d> jointPositions);

	/**
	* Checks if a point is within the bounds of a BoundingBox
	*
	* @param point XYZ coordinate point to check
	* @param bounds BoundingBox to check the point against
	* @return Whether or not the point given is within the bounds of the BoundingBox
	*/
	static bool withinBounds(Eigen::RowVector3d point, BoundingBox bounds);
	
};
#endif