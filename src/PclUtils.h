#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
#include "Ply.h"


#ifndef PCL_UTILS_H
#define PCL_UTILS_H

class PclUtils
{
public:
	static pcl::PointCloud<pcl::PointXYZ>::Ptr convertPlyToPointCloud(Ply pointCloud);
	static pcl::PCLPointCloud2 convertPlyToPointCloud2(Ply pointCloud);

	static void applyNearestNeighborFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud);

	static void generateMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud);

	static void generateTriangleMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud);

	static void outputToFile(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string fileName);
};
#endif
