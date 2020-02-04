//
// Created by Jeremy Keller
// 1/30/2020
//

#include "Ply.h"

using namespace std;
using namespace Eigen;
Ply::Ply() {
	pointCount = 0;
}

Ply::Ply(vector<RowVector4d> points) {
	this->points = points;
	pointCount = points.size();
}

Ply::Ply(vector<RowVector3d> points) {
	for (int i = 0; i < points.size(); i++) {
		// Convert 1x3 vector into a 1x4 vector with a fileIndex of 0
		RowVector4d convertedPoint = convertRowVector(points.at(i));
		this->points.push_back(convertedPoint);
	}
}

int Ply::getPointCount() {
	return pointCount;
}

void Ply::setPoints(vector<RowVector4d> points) {
	this->points = points;
	pointCount = points.size();
}

// Utils
void Ply::addPoint(Eigen::RowVector4d point) {
	points.push_back(point);
}

void Ply::addPoint(Eigen::RowVector3d point) {
	points.push_back(convertRowVector(point));
}
void Ply::addPoint(double x, double y, double z) {
	RowVector4d point;
	point(0) = x;
	point(1) = y;
	point(2) = z;
	point(3) = 0;
	points.push_back(point);
}

// Helper methods
RowVector4d convertRowVector(RowVector3d point) {
	RowVector4d convertedPoint;
	convertedPoint(0) = point(0);
	convertedPoint(1) = point(1);
	convertedPoint(2) = point(2);
	convertedPoint(3) = 0;
	return convertedPoint;
}
