#pragma once

#include <Eigen/Dense>
#include <string>

#include "IOUtils.h"
#include "Ply.h"

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
	static Ply applyTransforms(std::vector<Ply> pointClouds, std::unordered_map<std::string, Eigen::Matrix4Xd> transformations);
};
#endif MATRIX_UTILS_H

