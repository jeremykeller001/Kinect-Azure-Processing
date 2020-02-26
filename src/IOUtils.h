//
// IOUtils.h
// Utilities dealing with file i/o
//
// Created by: Jeremy Keller
// 2/06/2020
//

#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

#ifndef IO_UTILS_H
#define IO_UTILS_H

class IOUtils
{
private:
	static Eigen::RowVector4d readTransformLine(std::string line);

public:
	static bool endsWith(std::string const fileName, std::string const fileExtension);
	static std::vector<std::string> obtainMkvFilesFromDirectory(std::string dirPath);
	static std::unordered_map<std::string, Eigen::Matrix4Xd> readTransformationFile(std::string fileName);
};
#endif 
