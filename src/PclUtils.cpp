#include "PclUtils.h"
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/reconstruction.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/pcd_io.h>

void PclUtils::outputToFile(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string fileName) {
	pcl::io::savePCDFileASCII(fileName, *cloud);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclUtils::convertPlyToPointCloud(Ply pointCloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	cloud->points.resize(pointCloud.getPointCount());

	for (int i = 0; i < pointCloud.getPointCount(); i++) {
		Eigen::RowVector3d point = pointCloud.getPoints().at(i);
		cloud->push_back(pcl::PointXYZ(point(0), point(1), point(2)));
	}

	return cloud;
}

pcl::PCLPointCloud2 PclUtils::convertPlyToPointCloud2(Ply pointCloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = convertPlyToPointCloud(pointCloud);

	pcl::PCLPointCloud2 pc2;
	pcl::toPCLPointCloud2(*cloud, pc2);
	return pc2;
}

void PclUtils::applyNearestNeighborFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud) {
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;
	filter.setInputCloud(inCloud);
	filter.setRadiusSearch(0.5);
	filter.setMinNeighborsInRadius(3);
	filter.filter(*outCloud);

	return;
}

void PclUtils::generateMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud) {
	pcl::MeshConstruction<pcl::PointXYZ>::Ptr mesher;
	mesher->setInputCloud(inCloud);
	//mesher->reconstruct()
}

void PclUtils::generateTriangleMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud) {
	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(inCloud);
	n.setInputCloud(inCloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*inCloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	// Save file
	pcl::io::saveVTKFile("mesh.vtk", triangles);
}

