#include "MatrixUtils.h"

using namespace std;

Eigen::Matrix4d MatrixUtils::convertRowVector3ToMatrix4(Eigen::RowVector3d coordinate3) {
	Eigen::Matrix4d matrix;
	matrix(0, 0) = coordinate3(0);
	matrix(1, 0) = coordinate3(1);
	matrix(2, 0) = coordinate3(2);
	matrix(3, 0) = 1.0;

	return matrix;
}

Eigen::RowVector3d MatrixUtils::convertMatrix4ToRowVector3(Eigen::Matrix4d matrix) {
	Eigen::RowVector3d c3;
	c3(0) = matrix(0, 0);
	c3(1) = matrix(1, 0);
	c3(2) = matrix(2, 0);

	return c3;
}

Ply MatrixUtils::applySingleTransform(Ply pointCloud, Eigen::Matrix4Xd transform) {
	// Get points from the ply
	vector<Eigen::RowVector3d> coordinates = pointCloud.getPoints();
	
	// Convert 1x3 coordinates to 1x4 with a 1.0 as the last value
	vector<Eigen::Matrix4d> coordinates4;
	for (Eigen::RowVector3d coord : coordinates) {
		coordinates4.push_back(convertRowVector3ToMatrix4(coord));
	}

	// multiply each coordinate by the transform
	// TODO: Improve efficiency by using iterator
	for (int i = 0; i < coordinates4.size(); i++) {
		coordinates4.at(i) = transform * coordinates4.at(i);
	}

	// Convert coordinate back to 3d point
	coordinates.clear();
	for (Eigen::Matrix4d coord4 : coordinates4) {
		coordinates.push_back(convertMatrix4ToRowVector3(coord4));
	}

	pointCloud.setPoints(coordinates);
	return pointCloud;
}

vector<Ply> MatrixUtils::applyTransforms(vector<Ply> pointClouds, unordered_map<string, Eigen::Matrix4Xd> transformations) {
	// Match point clouds to transforms
	// If a point cloud cannot be matched to a transform, assume it does not need to be transformed

	// First check if transforms exist
	if (transformations.size() == 0) {
		return pointClouds;
	}

	// Initialize a new point cloud vector
	vector<Ply> transformedPlys;

	// Loop through point clouds
	for (Ply pc : pointClouds) {
		Ply transformedPc = Ply();
		for (auto it = transformations.begin(); it != transformations.end(); it++) {
			// For each pc, loop through transform and find matches
			if (IOUtils::endsWith(pc.getFileName(), it->first)) {
				transformedPc = applySingleTransform(pc, it->second);
				break;
			}
		}

		if (transformedPc.getPointCount() != 0) {
			transformedPlys.push_back(transformedPc);
		}
		else {
			// If no transform occured, just add the unmodified ply to the vector
			transformedPlys.push_back(pc);
		}
	}

	return transformedPlys;
}

vector<Eigen::RowVector3d> MatrixUtils::applyJointTrackingTransform(vector<Eigen::RowVector3d> jointPositions, Eigen::Matrix4Xd transform) {
	// Loop through tracked joints
	vector<Eigen::RowVector3d> jointPositionsTransformed;
	for (Eigen::RowVector3d jointPosition : jointPositions) {
		// Convert to 1x4 matrix
		Eigen::Matrix4d matrix;
		matrix(0, 0) = jointPosition(0);
		matrix(1, 0) = jointPosition(1);
		matrix(2, 0) = jointPosition(2);
		matrix(3, 0) = 1.0;

		// Apply transform
		matrix = transform * matrix;

		// Convert back to 3x1 matrix
		jointPositionsTransformed.push_back(convertMatrix4ToRowVector3(matrix));
	}

	return jointPositionsTransformed;
}