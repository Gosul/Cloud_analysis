// Main.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>

pcl::PointIndices::Ptr GetPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector4d& coeff)
{
	const double tol = 1e-5; //tolerance 
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000); //1000
	seg.setDistanceThreshold(0.01); //0.01
	// Segment the largest planar component from the remaining cloud
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);
		
	std::cerr << "Model coefficients: " << coefficients->values[0] << " "
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " "
		<< coefficients->values[3] << std::endl;
	
	coeff[0] = coefficients->values[0];
	coeff[1] = coefficients->values[1];
	coeff[2] = coefficients->values[2];
	coeff[3] = coefficients->values[3];

	return inliers;
}

int main(int argc, char** argv)
{ 
	//TODO:
	//float leafSize = std::stof(argv[1]);

	float leafSize = 0.05;//0.02

	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

	// Fill in the cloud data
	pcl::PCDReader reader;
	const std::string pathToInputFile = "C:/PCto2D/My_test_data/cafeteria_32.pcd";
	reader.read(pathToInputFile, *cloud); 

	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ")."<<"\n";

	// Create the filtering object
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(leafSize, leafSize, leafSize);
	sor.filter(*cloud_filtered);

	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";

	/*pcl::PCDWriter writer;
	const std::string pathToOutputFile = "C:/PCto2D/My_test_data/House 10MLN_downsampled.pcd";
	writer.write(pathToOutputFile, *cloud_filtered, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);*/
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(*cloud_filtered, *cloudXYZ);

	/*pcl::PointXYZ cloud_min_;
	pcl::PointXYZ cloud_max_;
	pcl::getMinMax3D(*cloudXYZ, cloud_min_, cloud_max_);*/
	
	pcl::visualization::CloudViewer viewer("PointCloud_Viewer");
	viewer.showCloud(cloudXYZ, "viewer_cloud");
	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	std::cerr << std::endl;
	for (int k = 1; k<=10; k++) // k<5
	{
		Eigen::Vector4d coeff;
		pcl::PointIndices::Ptr inliers = GetPlane(cloudXYZ, coeff);
		if (inliers==nullptr || inliers->indices.size() == 0)
		{
			std::cerr<<"Iter="<< k <<" Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
		// Extract the inliers
		extract.setInputCloud(cloudXYZ);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_plane);

		if (coeff[0]*coeff[0] + coeff[1]*coeff[1] < 1e-5)
		{
			std::cerr << "Iter=" << k << " PointCloud representing the planar component: " << cloud_plane->width * cloud_plane->height << " data points." << std::endl;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::copyPointCloud(*cloud_plane, *cloud_plane_rgb);
			uint32_t r = 255, g = 0, b = 0;
			int32_t rgb = (r << 16 | g << 8 | b);
			for (auto &p : cloud_plane_rgb->points) p.rgb = rgb;
			viewer.showCloud(cloud_plane_rgb, "viewer_plane" + std::to_string(k));
		}
		else 
		{
			std::cerr << "Iter=" << k << " Current extracted plane doesn't Z-plane." << std::endl;
		}
		extract.setNegative(true);
		extract.filter(*cloudXYZ);
	}

	while (!viewer.wasStopped()) {}
	return (0);
}
