#include <pcl/io/pcd_io.h>
#include <pcl/filters/uniform_sampling.h>
//#include <pcl/keypoints/uniform_sampling.h>

int
main(int argc, char** argv)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	// Uniform sampling object.
	pcl::UniformSampling<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	// We set the size of every voxel to be 1x1x1cm
	// (only one point per every cubic centimeter will survive).
	filter.setRadiusSearch(0.03f);
	
	filter.filter(*filteredCloud);
	// We need an additional object to store the indices of surviving points.
	pcl::PointCloud<int> keypointIndices;

	filter.compute(keypointIndices);
	pcl::copyPointCloud(*cloud, keypointIndices.points, *filteredCloud);
	pcl::io::savePCDFileASCII("uniform_sample.pcd",*filteredCloud);
}
