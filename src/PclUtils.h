#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "Ply.h"


#ifndef PCL_UTILS_H
#define PCL_UTILS_H

class PclUtils
{
public:
	static pcl::PointCloud<pcl::PointXYZ>::Ptr convertPlyToPointCloud(Ply pointCloud);
	static pcl::PCLPointCloud2 convertPlyToPointCloud2(Ply pointCloud);

	void applyNearestNeighborFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud);

	void generateMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud);

	void generateTriangleMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud);
};
#endif
