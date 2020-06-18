#pragma once

#include <Eigen/Dense>
#include <string>
#include <k4a/k4a.h>

#include "IOUtils.h"
#include "Ply.h"
#include "PclUtils.h"

#ifndef MATRIX_UTILS_H
#define MATRIX_UTILS_H

/**
* Static utility class for methods dealing with matrix calculations
*/
class MatrixUtils
{
private:

	/**
	* Converts a point cloud coordinate to a 4x1 matrix so it can be multiplied by a 4x4 transform.
	* The added coordinate will always evaluate to be 1.0.
	*
	* @param coordinate3 The xyz coordinate to be converted into xyz1 format
	* @return 4x1 matrix with the elements [x, y, z, 1]
	*/
	static Eigen::Matrix4d convertRowVector3ToMatrix4(Eigen::RowVector3d coordinate3);

	/**
	* Converts a 4x1 matrix back to an xyz coordinate.
	* The 4th coordinate will be removed.
	*
	* @param matrix 4x1 matrix to convert to xyz coordinate
	* @return XYZ coordinate
	*/
	static Eigen::RowVector3d convertMatrix4ToRowVector3(Eigen::Matrix4d matrix);

	/**
	* Applies a transform to a point cloud and returns the transformed Ply object
	*
	* @param pointCloud Point cloud set to transform
	* @param transform 4x4 matrix transform to apply to the point cloud
	* @return Transformed point cloud
	*/
	static Ply applySingleTransform(Ply pointCloud, Eigen::Matrix4Xd transform);

public:

	/**
	* Applies a set of transformations to a set of point clouds and returns a vector of the transformed clouds.
	*
	* @param pointClouds Set of point clouds to transform
	* @param transformations Map of transformations to apply, with the keys being the file suffix to apply the transformation to, 
	*	and the values being the 4x4 matrix transformation to apply 
	* @return Transformed point clouds
	*/
	static std::vector<Ply> applyTransforms(std::vector<Ply> pointClouds, std::unordered_map<std::string, Eigen::Matrix4Xd> transformations);

	/**
	* Applies a transform to a set of joint locations and returns the transformed joint locations
	* 
	* @param jointPositions Joint location xyz coordinate set to transform
	* @param transform 4x4 matrix transformation to apply to the joint locations
	* @return Transformed joint locations
	*/
	static std::vector<Eigen::RowVector3d> applyJointTrackingTransform(std::vector<Eigen::RowVector3d> jointPositions, Eigen::Matrix4Xd transform);
};
#endif MATRIX_UTILS_H

