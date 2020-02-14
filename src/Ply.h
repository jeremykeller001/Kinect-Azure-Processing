//
// Ply.h
// Represents a .ply file
// 
// Created by Jeremy Keller
// 1/30/2020
//

#pragma once
#include <string>
#include <vector>
#include <Eigen/Dense>

#ifndef PLY_H
#define PLY_H

class Ply
{
private:
	std::string fileName;

	// Indices:
	// x = 0; y = 1; z = 2;
	std::vector<Eigen::RowVector3d> points;

	// Number of points in this object
	int pointCount;

public:
	// Default constructor
	Ply();

	// {x y z} constructor
	Ply(std::vector<Eigen::RowVector3d> points);

	// Getters/Setters
	std::vector<Eigen::RowVector3d> getPoints();

	std::string getFileName();

	int getPointCount();

	void setFileName(std::string fileName);

	void setPoints(std::vector<Eigen::RowVector3d> points);

	// Utils
	void addPoint(Eigen::RowVector3d point);
	void addPoint(double x, double y, double z);

	void outputToFile(int frameIndex);
};

#endif