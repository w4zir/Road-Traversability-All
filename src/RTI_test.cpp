#include "include/prm.h"

#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

///** \brief Store cluster information. */
//struct cluster_stats
//{
//	float min_dist;
//	float max_dist;
//	int cluster_size;
//};

int main(int argc, char *argv[])
{
	if (argc < 1)
	{
		cerr << "usage: " << argv[0] << " input file to process." << endl;
		exit (EXIT_FAILURE);
	}
	std::string file_name = argv[1];

	pcl::PCDWriter writer;
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
	std::vector < std::vector <pcl::DEM<pcl::PointXYZ>::clusters> > dem_clusters;

	std::cout << "Starting process." 	<< std::endl;

	std::stringstream file;
	file << "/home/khan/phd_ws/road_synthetic_data/pointclouds/"<<file_name.c_str();
	reader.read (file.str(), *cloud_in);
	//	reader.read ("../data/dem6.pcd", *cloud_in);
	//	reader.read ("/home/khan/phd_ws/road_synthetic_data/pointclouds/road_obs10.pcd", *cloud_in);
	std::cout << "Points in cloud before filtering: " << cloud_in->points.size() << std::endl;


	// Create a set of planar coefficients with X=Y=0,Z=1
	pcl::ModelCoefficients::Ptr proj_plane_coefficients (new pcl::ModelCoefficients ());
	proj_plane_coefficients->values.resize (4);
	proj_plane_coefficients->values[0] = proj_plane_coefficients->values[1] = 0;
	proj_plane_coefficients->values[2] = 1.0;
	proj_plane_coefficients->values[3] = 0;

	pcl::PRM<pcl::PointXYZ> prmObj;
	prmObj.setInputCloud(cloud_in);
	prmObj.SetProjectedPlaneCoefficients(proj_plane_coefficients);
	prmObj.computePRM();

	std::vector< std::pair<int,int> > prm_graph;
	prmObj.getPRMAdjacencyList(prm_graph);

	int config_count = prmObj.getConfigurationX() * prmObj.getConfigurationY() * prmObj.getConfigurationTheta(); 

	std::string dataFolder = "/home/mudassir/phd_ws/data/traversability/pointclouds/kitti/2011_09_26_drive_0086/";
	std::string prmFolder = "/home/mudassir/phd_ws/data/traversability/prm/kitti/drive0086/v2/";
	std::string adjFolder = "/home/khan/phd_ws/road_ws/adjacency/";

	std::stringstream saveAdj;
	saveAdj << adjFolder.c_str() << "test" <<"_adj";
	std::ofstream saveAdjFile(saveAdj.str().c_str());
	if (saveAdjFile.is_open())
	{
		for (int i=0; i < prm_graph.size(); i++) 
		{
					saveAdjFile <<prm_graph[i].first<<" "<<prm_graph[i].second<<" "<<1<< "\n";
		}
//		saveAdjFile <<0<<" "<< config_count-1 <<" "<<0<< "\n";
		saveAdjFile.close();
		std::cout << "prm graph adjacency list saved to file successfully with points:" << prm_graph.size() << std::endl;
	}else {
		std::cout<<"Error: can not find directory"<<std::endl;
	}

	//	pcl::PointCloud<pcl::PointXYZ>::Ptr dem_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	//	pcl::DEM<pcl::PointXYZ> obj;
	//	
	//	obj.setInputCloud(cloud_in);
	//	//seg.setRoadPlaneCoefficients(model_coeff_pass);
	//	//seg.setRoadNormal(Eigen::Vector3f(model_coeff_pass->values[0],model_coeff_pass->values[1],model_coeff_pass->values[2]));
	//	obj.SetProjectedPlaneCoefficients(proj_plane_coefficients);
	//	obj.computeDEM();
	//	obj.getDEMClusters(dem_clusters);
	//	prmObj.getDEMCloud(*dem_cloud);

	//	std::cout << "clusters size" << dem_clusters.size() << std::endl;

	//	pcl::PointCloud <pcl::PointXYZRGB>::Ptr prm_cloud = prmObj.getPRMVisibilityCloud ();
	//	prm_cloud->height = 1;
	//	prm_cloud->width = prm_cloud->points.size();
	//	writer.write<pcl::PointXYZRGB> ("../data/prm_cloud.pcd", *prm_cloud, false);
	//
	//	pcl::PointCloud <pcl::PointXYZRGB>::Ptr dem_cloud = prmObj.getDEMVisibilityCloud ();
	//	dem_cloud->height = 1;
	//	dem_cloud->width = dem_cloud->points.size();
	//	writer.write<pcl::PointXYZRGB> ("../data/dem_cloud.pcd", *dem_cloud, false);
	//
	//	pcl::visualization::CloudViewer viewer ("Cluster viewer");
	//
	//	viewer.showCloud(dem_cloud);
	//	while (!viewer.wasStopped ())
	//	{
	//	}

	return 0;
}

