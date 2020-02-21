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

Ply::Ply(string fileNamee, vector<RowVector3d> pointss) {
	fileName = fileNamee;
	points = pointss;
	pointCount = points.size();
}

int Ply::getPointCount() {
	return pointCount;
}

void Ply::setPoints(vector<RowVector3d> points) {
	this->points = points;
	pointCount = points.size();
}

void Ply::setFileName(string fileNamee) {
	fileName = fileNamee;
}

//
// Utils
//
void Ply::addPoint(RowVector3d point) {
	points.push_back(point);
	this->pointCount++;
}
void Ply::addPoint(double x, double y, double z) {
	RowVector3d point;
	point(0) = x;
	point(1) = y;
	point(2) = z;
	points.push_back(point);
	this->pointCount++;
}

