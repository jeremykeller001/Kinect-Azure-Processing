#pragma once

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/reconstruction.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <vtkFillHolesFilter.h>
#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>
#include <vtkSmartPointer.h>
#include <vtkSmoothPolyDataFilter.h>
#include <string>
#include "Ply.h"


#ifndef PCL_UTILS_H
#define PCL_UTILS_H

/**
* Static methods related to PCL point cloud manipulation using PCL library methods
*/
class PclUtils
{
public:

	/**
	* Outputs a PCL point cloud to the specified file name
	*
	* @parma cloud Point cloud to output
	* @param fileName Name of the file to output. This can be a full file path. This must include the .pcd file extension.
	*/
	static void outputToFile(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string fileName);

	/**
	* Converts a Ply point cloud object to a PCL Point Cloud
	*
	* @param pointCloud Point cloud to convert
	* @return converted point cloud
	*/
	static pcl::PointCloud<pcl::PointXYZ>::Ptr convertPlyToPointCloud(Ply pointCloud);

	/**
	* Converts a PCL point cloud back to a Ply point cloud object
	*
	* @param cloud Point cloud to convert
	* @return Converted point cloud
	*/
	static Ply convertPcdToPly(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	/**
	* Applies a statistical outlier filter to a point cloud, removing outlier points
	*
	* @param inCloud Point cloud to filter
	* @param outCloud Container which will receive the filtered point cloud
	*/
	static void applyStatisticalOutlierFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud);

	/**
	* Downsamples a point cloud, resamples it using a moving least squares algorithm, then outputs the point cloud and generates a mesh .obj output for it
	*
	* cloud Point cloud to process
	* outputName Full file path with name, no file extension
	* skipMesh Whether or not the mesh output should be skipped. Skipping it will drastically reduce processing time.
	*/
	static void resampleAndMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string outputName, bool skipMesh);

	/**
	* Downsamples a point cloud. Useful to even out the point density variance of multiple Kinect cameras with differing distances from the capture subject
	*
	* @param inCloud Point cloud to downsample
	* @param outCloud Container which will receive the downsampled point cloud
	*/
	static void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud);
};
#endif
