#include "PclUtils.h"

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

Ply PclUtils::convertPcdToPly(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	Ply outCloud;
	for (pcl::PointXYZ point : cloud->points) {
		outCloud.addPoint({ point.x, point.y, point.z });
	}

	return outCloud;
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

void PclUtils::applyStatisticalOutlierFilter(Ply cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud = PclUtils::convertPlyToPointCloud(cloud);

	PclUtils::applyStatisticalOutlierFilter(pclCloud, filteredCloud);
	delete& pclCloud;
}

void PclUtils::applyStatisticalOutlierFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud) {
	//std::cerr << "Cloud before filtering: " << std::endl;
	//std::cerr << *inCloud << std::endl;
	
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(inCloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(2.0);
	sor.filter(*outCloud);

	//std::cerr << "Cloud after filtering: " << std::endl;
	//std::cerr << *outCloud << std::endl;
}

void PclUtils::filterAndMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string outputName) {

	// Moving Least Squares Filtering
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
	mls.setInputCloud(cloud);
	mls.setSearchRadius(30.0); // 30mm search radius
	mls.setPolynomialFit(true);
	mls.setPolynomialOrder(2);
	//mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
	//mls.setUpsamplingRadius(15.0); // 15mm upsampling
	//mls.setUpsamplingStepSize(8.0); // 8mm step size

	pcl::PointCloud<pcl::PointXYZ>::Ptr mlsCloud(new pcl::PointCloud<pcl::PointXYZ>);
	mls.process(*mlsCloud);

	std::string mlsOutputName = std::string(outputName);
	mlsOutputName += ".pcd";
	outputToFile(mlsCloud, mlsOutputName);

	// Normal Estimation
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setNumberOfThreads(16);
	ne.setInputCloud(mlsCloud);
	ne.setRadiusSearch(30.0); // 30mm search radius
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*mlsCloud, centroid);
	ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

	pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>());
	ne.compute(*cloudNormals);

	for (size_t i = 0; i < cloudNormals->size(); i++) {
		cloudNormals->points[i].normal_x *= -1;
		cloudNormals->points[i].normal_y *= -1;
		cloudNormals->points[i].normal_z *= -1;
	}

	pcl::PointCloud<pcl::PointNormal>::Ptr smoothedNormalsCloud(new pcl::PointCloud<pcl::PointNormal>());
	pcl::concatenateFields(*mlsCloud, *cloudNormals, *smoothedNormalsCloud);

	// Meshing
	pcl::Poisson<pcl::PointNormal> poisson;
	poisson.setDepth(9); // TODO: Re-evaluate this param
	poisson.setInputCloud(smoothedNormalsCloud);
	pcl::PolygonMesh mesh;
	poisson.reconstruct(mesh);
	std::string meshOutputName = std::string(outputName);
	meshOutputName += ".obj";
	pcl::io::saveOBJFile(meshOutputName, mesh);
}

void PclUtils::resampleAndMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	
	//
	// Smoothing
	//
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
	mls.setSearchRadius(30);

	// Reconstruct
	mls.process(*mls_points);

	// Save output
	pcl::io::savePCDFile("normals.pcd", *mls_points);

	//
	// Meshing
	//
	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(mls_points);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(30);

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

	pcl::io::saveOBJFile("mesh.obj", triangles);

	return;
}

void PclUtils::downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud) {
	// Voxel Grid filtering
	pcl::VoxelGrid<pcl::PointXYZ> vox;
	vox.setInputCloud(inCloud);
	vox.setLeafSize(10.0, 10.0, 10.0);
	std::cerr << "PointCloud before voxel grid filtering: " << inCloud->width * inCloud->height << std::endl;
	vox.filter(*outCloud);
	std::cerr << "PointCloud after voxel grid filtering: " << outCloud->width * outCloud->height << std::endl;
	pcl::io::savePCDFile("voxel_grid.pcd", *outCloud);
}
