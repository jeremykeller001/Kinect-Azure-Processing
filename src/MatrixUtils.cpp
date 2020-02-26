#include "MatrixUtils.h"

using namespace std;

Eigen::RowVector4d MatrixUtils::convertRowVector3To4(Eigen::RowVector3d coordinate3) {
	Eigen::RowVector4d c4;
	c4(0) = coordinate3(0);
	c4(1) = coordinate3(1);
	c4(2) = coordinate3(2);
	c4(3) = 1.0;

	return c4;
}

Eigen::RowVector3d MatrixUtils::convertRowVector4To3(Eigen::RowVector4d coordinate4) {
	Eigen::RowVector3d c3;
	c3(0) = coordinate4(0);
	c3(1) = coordinate4(1);
	c3(2) = coordinate4(2);

	return c3;
}

Ply MatrixUtils::applySingleTransform(Ply pointCloud, Eigen::Matrix4Xd transform) {
	// Get points from the ply
	vector<Eigen::RowVector3d> coordinates = pointCloud.getPoints();
	
	// Convert 1x3 coordinates to 1x4 with a 1.0 as the last value
	vector<Eigen::RowVector4d> coordinates4;
	for (Eigen::RowVector3d coord : coordinates) {
		coordinates4.push_back(convertRowVector3To4(coord));
	}

	// multiply each coordinate by the transform
	for (int i = 0; i < coordinates4.size(); i++) {
		coordinates4.at(i) = coordinates4.at(i) * transform;
	}

	// Convert coordinate back to 3d point
	coordinates.clear();
	for (Eigen::RowVector4d coord4 : coordinates4) {
		coordinates.push_back(convertRowVector4To3(coord4));
	}

	pointCloud.setPoints(coordinates);
	return pointCloud;
}

Ply MatrixUtils::applyTransforms(vector<Ply> pointClouds, unordered_map<string, Eigen::Matrix4Xd> transformations) {
	// Match point clouds to transforms
	// If a point cloud cannot be matched to a transform, assume it does not need to be transformed

	// Initialize new point cloud
	Ply combinedPc = Ply();

	// Loop through point clouds
	for (Ply pc : pointClouds) {
		for (auto it = transformations.begin(); it != transformations.end(); it++) {
			// For each pc, loop through transform and find matches
			if (IOUtils::endsWith(pc.getFileName(), it->first)) {
				combinedPc.merge(applySingleTransform(pc, it->second));
				break;
			}
		}
	}

	return Ply();
}