//
// Created by Jeremy Keller
// 1/30/2020
//

#include "Ply.h"

using namespace std;
using namespace Eigen;
Ply::Ply() {
	fileName = "";
	points = vector<RowVector3d>();
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

void Ply::setPoints(vector<RowVector3d> pointss) {
	points = pointss;
	pointCount = points.size();
}

void Ply::setFileName(string fileNamee) {
	fileName = fileNamee;
}

vector<Eigen::RowVector3d> Ply::getPoints() {
	return points;
}

string Ply::getFileName() {
	return fileName;
}

//
// Utils
//
void Ply::addPoint(RowVector3d point) {
	points.push_back(point);
	pointCount++;
}
void Ply::addPoint(double x, double y, double z) {
	RowVector3d point;
	point(0) = x;
	point(1) = y;
	point(2) = z;
	points.push_back(point);
	pointCount++;
}

void Ply::merge(Ply plyToMerge) {
	for (RowVector3d point : plyToMerge.getPoints()) {
		addPoint(point);
	}
}

void Ply::outputToFile(int frameIndex, string directory) {
	stringstream outputFullPath;
	outputFullPath << directory << "\\" << frameIndex << ".ply";

	ofstream out;
	out.open(outputFullPath.str(), ios::out);
	out << "ply" << endl;
	out << "format ascii 1.0" << endl;
	out << "element vertex " << pointCount << endl;
	out << "property float x" << endl;
	out << "property float y" << endl;
	out << "property float z" << endl;
	out << "end_header" << endl;

	for (RowVector3d point : points) {
		out << point(0) << " " << point(1) << " " << point(2) << endl;
	}

	out.close();
}

void Ply::outputToPcd(int frameIndex, string directory) {
	stringstream outputFullPath;
	outputFullPath << directory << "\\" << frameIndex << ".pcd";

	ofstream out;
	out.open(outputFullPath.str(), ios::out);
	out << "# .PCD v.7 ? Point Cloud Data file format" << endl;
	out << "VERSION .7" << endl;
	out << "TYPE F F F" << endl;
	out << "WIDTH " << pointCount << endl;
	out << "HEIGHT 1" << endl;
	out << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
	out << "POINTS " << pointCount << endl;
	out << "DATA ascii" << endl;

	for (RowVector3d point : points) {
		out << point(0) << " " << point(1) << " " << point(2) << endl;
	}

	out.close();
}

