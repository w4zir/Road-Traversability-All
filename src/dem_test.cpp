#include "include/dem.h"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>


int main()
{
	pcl::PCDWriter writer;
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
	std::vector < std::vector <pcl::DEM<pcl::PointXYZ>::clusters> > dem_clusters;
	
	std::cout << "Starting process." 	<< std::endl;

	reader.read ("/home/wazir/phd_ws/traversability/pointclouds/clearance_roads/road_clear3.pcd", *cloud_in);
	std::cout << "Points in cloud before filtering: " << cloud_in->points.size() << std::endl;


	// Create a set of planar coefficients with X=Y=0,Z=1
	pcl::ModelCoefficients::Ptr proj_plane_coefficients (new pcl::ModelCoefficients ());
	proj_plane_coefficients->values.resize (4);
	proj_plane_coefficients->values[0] = proj_plane_coefficients->values[1] = 0;
	proj_plane_coefficients->values[2] = 1.0;
	proj_plane_coefficients->values[3] = 0;

//	pcl::PRM<pcl::PointXYZ> prmObj;
//	prmObj.setInputCloud(cloud_in);
//	prmObj.SetProjectedPlaneCoefficients(proj_plane_coefficients);
//	prmObj.computePRM();

	pcl::PointCloud<pcl::PointXYZ>::Ptr dem_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::DEM<pcl::PointXYZ> obj;
	
	obj.setInputCloud(cloud_in);
	//seg.setRoadPlaneCoefficients(model_coeff_pass);
	//seg.setRoadNormal(Eigen::Vector3f(model_coeff_pass->values[0],model_coeff_pass->values[1],model_coeff_pass->values[2]));
	obj.SetProjectedPlaneCoefficients(proj_plane_coefficients);
	obj.computeDEM();
	obj.getDEMClusters(dem_clusters);
//	prmObj.getDEMCloud(*dem_cloud);
//			
//	std::cout << "clusters size" << dem_clusters.size() << std::endl;

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = obj.getDEMVisibilityCloud ();
	//  writer.write<pcl::PointXYZRGB> ("road_rgb.pcd", *colored_cloud, false);

	pcl::visualization::CloudViewer viewer ("Cluster viewer");

	viewer.showCloud(colored_cloud);
	while (!viewer.wasStopped ())
	{
	}

	return 0;
}

