//
// BodyTrackingUtils.cpp
// Custom Utilities dealing with the Kinect Body Tracking SDK
// Adopted from code originally written by Eddy.Rogers@du.edu
//
// Created by: Jeremy Keller
// 3/16/2020
//

#include "BodyTrackingUtils.h"

using namespace std;

bool BodyTrackingUtils::predictJoints(int frameCount, k4abt_tracker_t tracker, k4a_capture_t capture_handle, vector<Eigen::RowVector3d>* jointPositions) {
	k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, capture_handle, K4A_WAIT_INFINITE);
	if (queue_capture_result != K4A_WAIT_RESULT_SUCCEEDED)
	{
		cerr << "Error! Adding capture to tracker process queue failed!" << endl;
		return false;
	}

	k4abt_frame_t body_frame = nullptr;
	k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
	if (pop_frame_result != K4A_WAIT_RESULT_SUCCEEDED)
	{
		cerr << "Error! Popping body tracking result failed!" << endl;
		return false;
	}

	size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
	if (num_bodies > 1) {
		cerr << "Warning: More than one body detected in group: " << frameCount << 
			". Skipping body tracking processing for group." << endl;
		return false;
	}
	else if (num_bodies == 0) {
		cerr << "Warning: No body detected in group: " << frameCount <<
			". Skipping body tracking processing for group." << endl;
	}

	for (size_t i = 0; i < num_bodies; i++)
	{
		k4abt_skeleton_t skeleton;
		if (k4abt_frame_get_body_skeleton(body_frame, i, &skeleton) != K4A_RESULT_SUCCEEDED) {
			cerr << "Error: Unable to obtain body from body frame in group: " << frameCount << endl;
			return false;
		}

		int body_id = k4abt_frame_get_body_id(body_frame, i);

		int total_confidence = 0;

		for (int j = 0; j < (int)K4ABT_JOINT_COUNT; j++)
		{
			int confidenceLevel = skeleton.joints[j].confidence_level;
			if (confidenceLevel == 0 || skeleton.joints[j].position.xyz.z > 10000 || skeleton.joints[j].position.xyz.z < 0) {
				// Do not process joint with confidence level of 0
				continue;
			}
			Eigen::RowVector3d jointPosition;
			jointPosition(0) = skeleton.joints[j].position.xyz.x;
			jointPosition(1) = skeleton.joints[j].position.xyz.y;
			jointPosition(2) = skeleton.joints[j].position.xyz.z;
			jointPositions->push_back(jointPosition);
		}
	}
	k4abt_frame_release(body_frame);

	if (num_bodies == 1) {
		return true;
	}
	return false;
}

BodyTrackingUtils::BoundingBox BodyTrackingUtils::createBoundingBox(vector<Eigen::RowVector3d> jointPositions) {
	BodyTrackingUtils::BoundingBox bounds;
	bounds.xMin = 999999.9;
	bounds.xMax = -999999.9;
	bounds.yMin = 999999.9;
	bounds.yMax = -999999.9;
	bounds.zMin = 999999.9;
	bounds.zMax = -999999.9;

	for (Eigen::RowVector3d jointPosition : jointPositions) {
		// Validate coordinates. If they are invalid positions, then do not process coordinate
		if (jointPosition(2) < 0 || jointPosition(2) > 10000) {
			continue;
		}

		// X
		if (jointPosition(0) < bounds.xMin) {
			bounds.xMin = jointPosition(0);
		} else if (jointPosition(0) > bounds.xMax) {
			bounds.xMax = jointPosition(0);
		}

		// Y
		if (jointPosition(1) < bounds.yMin) {
			bounds.yMin = jointPosition(1);
		}
		else if (jointPosition(1) > bounds.yMax) {
			bounds.yMax = jointPosition(1);
		}

		// Z
		if (jointPosition(2) < bounds.zMin) {
			bounds.zMin = jointPosition(2);
		}
		else if (jointPosition(2) > bounds.zMax) {
			bounds.zMax = jointPosition(2);
		}
	}
	
	// Length in mm to allow deviations from bounds 
	// Bounds are calculated at joint angle positions, not outer hull locations, 
	//	so we need to modify it a bit to capture the outside of the body
	double boundsMod = 150;
	bounds.xMin -= boundsMod;
	bounds.xMax += boundsMod;
	bounds.yMin -= boundsMod;
	bounds.yMax += boundsMod;
	bounds.zMin -= boundsMod;
	bounds.zMax += boundsMod;

	return bounds;
}

bool BodyTrackingUtils::withinBounds(Eigen::RowVector3d point, BoundingBox bounds) {
	return ((point(0) >= bounds.xMin && point(0) <= bounds.xMax)
		&& (point(1) >= bounds.yMin && point(1) <= bounds.yMax)
		&& (point(2) >= bounds.zMin && point(2) <= bounds.zMax));
}