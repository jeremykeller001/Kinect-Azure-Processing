#pragma once

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/reconstruction.h>
#include <pcl/surface/poisson.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <vtkFillHolesFilter.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <string>
#include "Ply.h"


#ifndef PCL_UTILS_H
#define PCL_UTILS_H

class PclUtils
{
public:
	static pcl::PointCloud<pcl::PointXYZ>::Ptr convertPlyToPointCloud(Ply pointCloud);
	static pcl::PCLPointCloud2 convertPlyToPointCloud2(Ply pointCloud);
	static Ply convertPcdToPly(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	static void applyStatisticalOutlierFilter(Ply inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud);

	static void applyStatisticalOutlierFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud);

	static void resampleAndMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string outputName, bool skipMesh);

	static void outputToFile(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string fileName);

	static void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud);
};
#endif
