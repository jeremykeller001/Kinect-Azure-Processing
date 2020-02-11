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


public:
	void applyTransform(Ply pointCloud, Eigen::Matrix4Xd transform);
	void isolateBody(Ply pointCloud, Eigen::RowVector3d minimums, Eigen::RowVector3d maximums);

	// TODO:
	void applyMultipleTransform(std::vector<Ply> pointCloud, std::string transformFile);
};
#endif MATRIX_UTILS_H

