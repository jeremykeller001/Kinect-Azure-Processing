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
public:
	typedef struct Transform4x4 {
		std::string fileName;
		Eigen::Matrix4Xd transform;
	} Transform4x4;

	static std::vector<std::string> obtainMkvFilesFromDirectory(std::string dirPath);
	static std::vector<Transform4x4> readTransformationFile(std::string fileName);
};
#endif 

