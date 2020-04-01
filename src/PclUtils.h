#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/reconstruction.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
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

	static void applyStatisticalOutlierFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud);

	static void resample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	static void generateMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud);

	static void generateTriangleMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud);

	static void outputToFile(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string fileName);
};
#endif
