#include "PclUtils.h"
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/reconstruction.h>

void PclUtils::outputToFile(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string fileName) {
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>(fileName, *cloud, false);

}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclUtils::convertPlyToPointCloud(Ply pointCloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	for (Eigen::RowVector3d point : pointCloud.getPoints()) {
		cloud->points.push_back({ (float) point(0), (float) point(1), (float) point(2) });
	}

	std::cerr << "Completed pcd conversion" << std::endl;
	return cloud;
}

pcl::PCLPointCloud2 PclUtils::convertPlyToPointCloud2(Ply pointCloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = convertPlyToPointCloud(pointCloud);

	pcl::PCLPointCloud2 pc2;
	pcl::toPCLPointCloud2(*cloud, pc2);
	return pc2;
}

void PclUtils::applyNearestNeighborFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud) {
	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *inCloud << std::endl;
	
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;
	filter.setInputCloud(inCloud);
	filter.setRadiusSearch(15);
	filter.setMinNeighborsInRadius(4);
	filter.filter(*outCloud);

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *outCloud << std::endl;

	return;
}

void PclUtils::applyStatisticalOutlierFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud) {
	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *inCloud << std::endl;
	
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(inCloud);
	sor.setMeanK(500);
	sor.setStddevMulThresh(0.75);
	sor.filter(*outCloud);

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *outCloud << std::endl;

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("filtered_inliers_500_0.75.pcd", *outCloud, false);

	sor.setNegative(true);
	sor.filter(*outCloud);
	writer.write<pcl::PointXYZ>("filtered_outliers_500_0.75.pcd", *outCloud, false);
}


void PclUtils::resample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	// Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

	mls.setComputeNormals(true);

	// Set parameters
	mls.setInputCloud(cloud);
	mls.setPolynomialOrder(2);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(15);

	// Reconstruct
	mls.process(*mls_points);

	// Save output
	pcl::io::savePCDFile("normals.pcd", *mls_points);


	// Meshing
	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(mls_points);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(25);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(mls_points);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	pcl::io::saveOBJFile("mesh_240.obj", triangles);

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

