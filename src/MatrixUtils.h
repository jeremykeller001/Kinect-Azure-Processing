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
	// Converts a point cloud coordinate to a 1x4 matrix so it can be multiplied by a 4x4 transform
	Eigen::RowVector4d convertRowVector3To4(Eigen::RowVector3d coordinate3);

	Eigen::RowVector3d convertRowVector4To3(Eigen::RowVector4d coordinate4);

	Ply applySingleTransform(Ply pointCloud, Eigen::Matrix4Xd transform);

public:
	Ply applyTransforms(std::vector<Ply> pointClouds, std::unordered_map<std::string, Eigen::Matrix4Xd> transformations);
	//void isolateBody(Ply pointCloud, Eigen::RowVector3d minimums, Eigen::RowVector3d maximums);

	// TODO:
	//void applyMultipleTransform(std::vector<Ply> pointCloud, std::string transformFile);
};
#endif MATRIX_UTILS_H

