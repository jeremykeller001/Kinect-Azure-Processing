#pragma once

#include <Eigen/Dense>
#include <string>
#include <k4a/k4a.h>

#include "IOUtils.h"
#include "Ply.h"
#include "PclUtils.h"

#ifndef MATRIX_UTILS_H
#define MATRIX_UTILS_H

class MatrixUtils
{
private:
	// Converts a point cloud coordinate to a 4x1 matrix so it can be multiplied by a 4x4 transform
	static Eigen::Matrix4d convertRowVector3ToMatrix4(Eigen::RowVector3d coordinate3);

	static Eigen::RowVector3d convertMatrix4ToRowVector3(Eigen::Matrix4d matrix);

	static Ply applySingleTransform(Ply pointCloud, Eigen::Matrix4Xd transform);

public:
	static std::vector<Ply> applyTransforms(std::vector<Ply> pointClouds, std::unordered_map<std::string, Eigen::Matrix4Xd> transformations);
	static std::vector<Eigen::RowVector3d> applyJointTrackingTransform(std::vector<Eigen::RowVector3d> jointPositions, Eigen::Matrix4Xd transform);
};
#endif MATRIX_UTILS_H

