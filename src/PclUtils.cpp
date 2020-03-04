#include "PclUtils.h"
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/reconstruction.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr convertPlyToPointCloud(Ply pointCloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

	for (Eigen::RowVector3d point : pointCloud.getPoints()) {
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

}

