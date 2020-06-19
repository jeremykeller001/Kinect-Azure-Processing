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
#include <sstream>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

#ifndef PLY_H
#define PLY_H

/**
* Container representing a point cloud in .ply format
*/
class Ply
{
private:
	/**
	* File name of the ply
	*/
	std::string fileName;

	/**
	* XYZ points in the point cloud
	*/
	std::vector<Eigen::RowVector3d> points;

	/**
	* Number of points contained within the cloud
	*/
	int pointCount;

public:
	/**
	* Default constructor
	*/
	Ply();

	/**
	* Constructor taking in a file name and an existing vector of points
	*
	* @param fileName File name of the point cloud
	* @param points XYZ points of the point cloud
	*/
	Ply(std::string fileName, std::vector<Eigen::RowVector3d> points);

	//
	// Getters/Setters
	//
	std::vector<Eigen::RowVector3d> getPoints();

	std::string getFileName();

	int getPointCount();

	void setFileName(std::string fileName);

	void setPoints(std::vector<Eigen::RowVector3d> points);

	//
	// Utils
	//
	/**
	* Adds a point to the cloud
	*
	* @param point XYZ point to add
	*/
	void addPoint(Eigen::RowVector3d point);

	/**
	* Adds a point to the cloud
	*
	* @param x x-coordinate of the point to add
	* @param y y-coordinate of the point to add
	* @param z z-coordinate of the point to add
	*/
	void addPoint(double x, double y, double z);

	/**
	* Outputs this to a local file with the given file name and directory
	*
	* @param fileName Name of the file to output, without the trailing .ply extension
	* @param directory Directory to place the file in
	*/
	void outputToFile(std::string fileName, std::string directory);

	/**
	* Outputs this cloud to .pcd format, with the file name being the frame index
	*
	* @param frameIndex Index of the point cloud, this will be the output name
	* @param directory Directory to place the file in
	*/
	void outputToPcd(int frameIndex, std::string directory);

	/**
	* Merges another Ply object with this object
	*
	* @param plyToMerge Point cloud to merge with this object
	*/
	void merge(Ply plyToMerge);
};

#endif