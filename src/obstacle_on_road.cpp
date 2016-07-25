#include "include/obstacle_inclusion.h"

#include <ctime>
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

int main(int argc, char *argv[])
{
	srand ((unsigned int) time (NULL));
	if (argc < 4)
	{
		cerr << "usage: " << argv[0] << "input file, output file, obstacle count." << endl;
		exit (EXIT_FAILURE);
	}
	std::string inFile = argv[1];
	std::string opFile = argv[2];
	int obstacle_count;
	std::istringstream (argv[3]) >> obstacle_count;

	pcl::PCDWriter writer;
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_op (new pcl::PointCloud<pcl::PointXYZ>);

	Eigen::MatrixXf obstacle_info;

	std::cout << "Starting process." 	<< std::endl;

	std::stringstream inStream;
	inStream << "/home/wazir/phd_ws/traversability/pointclouds/clearance_roads/"<<inFile.c_str();
	reader.read (inStream.str(), *cloud_in);
	std::cout << "Points in cloud before filtering: " << cloud_in->points.size() << std::endl;

	for (int i=2; i<= 100; i++)
	{
		pcl::OBSTACLE_INCLUSION<pcl::PointXYZ> obj;
		obj.setInputCloud(cloud_in);
		obj.setObstacleCount(obstacle_count);
		obj.samePairObstacles();
		obj.getOutputCloud(cloud_op);
		obj.getObstaclesInfo(obstacle_info);

		std::cout << "obstacle info \t" << obstacle_info.rows() << std::endl;

		std::stringstream opStream;
		opStream << "/home/wazir/phd_ws/traversability/pointclouds/clearance_roads/" << opFile.substr(0,opFile.find(".pcd")) << i <<".pcd";
		writer.write<pcl::PointXYZ> (opStream.str(), *cloud_op, false);

		/*
		 * save obstacles info in a text file
		 */
		std::stringstream saveObs;
		saveObs << "/home/wazir/phd_ws/traversability/pointclouds/clearance_roads/" << opFile.substr(0,opFile.find(".pcd")) << i <<"_obs";
		std::ofstream saveObsFile(saveObs.str().c_str());
		if (saveObsFile.is_open())
		{
			for (int i=0; i < obstacle_info.rows(); i++) 
			{
				saveObsFile <<obstacle_info.row(i) <<" " << -1 <<"\n";
			}
			saveObsFile.close();
		}else {
			std::cout<<"Error: can not find directory"<<std::endl;
		}
	}
	//	writer.write<pcl::PointXYZ> ("/home/khan/phd_ws/traversability/pointclouds/complex_routes/road_long02.pcd", *cloud_op, false);

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

	/*
	 * visualize output cloud
	 */
	//	pcl::visualization::CloudViewer viewer ("Cluster viewer");
	//
	//	viewer.showCloud(cloud_op);
	//	while (!viewer.wasStopped ())
	//	{
	//	}

	return 0;
}
