//
// Ply.h
// Represents a .ply file
// 
// Created by Jeremy Keller
// 1/30/2020
//

#pragma once
#include <vector>
#include <Eigen/Dense>

#ifndef PLY_H
#define PLY_H

class Ply
{
private:
	// Indices:
	// x = 0; y = 1; z = 2; fileIndex = 3
	std::vector<Eigen::RowVector4d> points;

	// Number of points in this object
	int pointCount;

public:
	// Default constructor
	Ply();

	// {x y z} constructor
	// A file index of 0 will be assigned
	Ply(std::vector<Eigen::RowVector3d> points);

	// {x y z fileIndex} constructor
	Ply(std::vector<Eigen::RowVector4d> points);

	// Getters/Setters
	std::vector<Eigen::RowVector4d> getPoints();

	int getPointCount();

	void setPoints(std::vector<Eigen::RowVector4d> points);

	// Utils
	void addPoint(Eigen::RowVector4d point);
	void addPoint(Eigen::RowVector3d point);
	void addPoint(double x, double y, double z);
	void recalculatePointCount();



};

#endif