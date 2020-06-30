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

Ply PclUtils::convertPcdToPly(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	Ply outCloud;
	for (pcl::PointXYZ point : cloud->points) {
		outCloud.addPoint({ point.x, point.y, point.z });
	}

	return outCloud;
}

void PclUtils::applyStatisticalOutlierFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud) {
	//std::cerr << "Cloud before filtering: " << std::endl;
	//std::cerr << *inCloud << std::endl;
	
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(inCloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*outCloud);

	//std::cerr << "Cloud after filtering: " << std::endl;
	//std::cerr << *outCloud << std::endl;
}

void PclUtils::resampleAndMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string outputName, bool skipMesh, Eigen::RowVector3d centroidCoordinates) {
	//
	// Downsample for performance
	//
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	downsample(cloud, cloud2);

	//
	// Smoothing
	//
	// Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	// Create output cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Init mls object
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
	// Set parameters
	mls.setInputCloud(cloud2);
	mls.setNumberOfThreads(atoi(std::getenv("NUMBER_OF_PROCESSORS")));
	mls.setPolynomialOrder(2);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(25); // Smoothing radius. Higher values will result in a smoother point cloud, but more points removed
	// Reconstruct
	mls.process(*smoothedCloud);

	// Filter outlying points
	pcl::PointCloud<pcl::PointXYZ>::Ptr smoothedFilteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;
	filter.setInputCloud(smoothedCloud);
	filter.setRadiusSearch(15.0); // Search radius in mm. Valid neighbors must be within this radius of the original point
	filter.setMinNeighborsInRadius(8); // If there are less than this number of neighbors within the search radius of the original point, it will be filtered 
	filter.filter(*smoothedFilteredCloud);

	// Save output
	pcl::io::savePCDFile(outputName + ".pcd", *smoothedFilteredCloud);

	//
	// Meshing
	//
	// Create search tree*
	if (skipMesh) {
		return;
	}

	// Calculate normals
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	ne.setNumberOfThreads(atoi(std::getenv("NUMBER_OF_PROCESSORS")));
	ne.setInputCloud(smoothedFilteredCloud);
	ne.setRadiusSearch(50);
	ne.setViewPoint(centroidCoordinates(0), centroidCoordinates(1), centroidCoordinates(2));
	ne.compute(*normals);
	// Flip normals direction since viewpoint was placed inside subject
	for (int i = 0; i < normals->size(); i++) {
		normals->points[i].normal_x *= -1;
		normals->points[i].normal_y *= -1;
		normals->points[i].normal_z *= -1;
	}
	// Combine XYZ points and normals
	pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*smoothedFilteredCloud, *normals, *pointNormals);

	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(pointNormals);

	// Initialize meshing objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gpt;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gpt.setSearchRadius(50);

	// Set typical values for the parameters
	gpt.setMu(2.5);
	gpt.setMaximumNearestNeighbors(100);
	gpt.setMaximumSurfaceAngle(2 * M_PI / 3); // 120 degrees
	gpt.setMinimumAngle(M_PI / 6); // 30 degrees
	gpt.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gpt.setNormalConsistency(false);

	// Get result
	gpt.setInputCloud(pointNormals);
	gpt.setSearchMethod(tree2);
	gpt.reconstruct(triangles);

	// Hole Filling
	vtkSmartPointer<vtkPolyData> input;
	pcl::VTKUtils::mesh2vtk(triangles, input);

	vtkSmartPointer<vtkFillHolesFilter> fillHolesFilter =
		vtkSmartPointer<vtkFillHolesFilter>::New();

	fillHolesFilter->SetInputData(input);
	fillHolesFilter->SetHoleSize(50.0); // Fill holes up to 50mm
	fillHolesFilter->Update();

	// Make the triangle winding order consistent now that hole filling is complete
	vtkSmartPointer<vtkPolyDataNormals> normals2 =
		vtkSmartPointer<vtkPolyDataNormals>::New();
	normals2->SetInputData(fillHolesFilter->GetOutput());
	normals2->ConsistencyOn();
	normals2->SplittingOff();
	normals2->Update();

	vtkSmartPointer<vtkSmoothPolyDataFilter> smoothFilter =
		vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
	smoothFilter->SetNumberOfIterations(100);
	smoothFilter->SetInputData(normals2->GetOutput());
	smoothFilter->SetRelaxationFactor(0.05);
	smoothFilter->FeatureEdgeSmoothingOff();
	smoothFilter->BoundarySmoothingOn();
	smoothFilter->Update();

	pcl::VTKUtils::vtk2mesh(smoothFilter->GetOutput(), triangles);
	
	pcl::io::saveOBJFile(outputName + ".obj", triangles);

	return;
}

void PclUtils::downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud) {
	// uniform sampling filter
	pcl::UniformSampling<pcl::PointXYZ> filter;
	filter.setInputCloud(inCloud);
	filter.setRadiusSearch(7.5); // Radius to create downsampling boxes in millimeters. Higher values will downsample more points
	//std::cerr << "PointCloud before filtering: " << inCloud->width * inCloud->height << std::endl;
	filter.filter(*outCloud);
	//std::cerr << "PointCloud after filtering: " << outCloud->width * outCloud->height << std::endl;
	//pcl::io::savePCDFile("downsampled.pcd", *outCloud);
}
