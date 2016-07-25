#include "include/prm.h"

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

double get_cpu_time(){
	return clock() / CLOCKS_PER_SEC;
}

int main(int argc, char *argv[])
{
	pcl::PCDReader reader;
	pcl::PCDWriter writer;

	std::clock_t    start;


	//		std::string pointcloudFolder = "/home/khan/phd_ws/traversability/pointclouds/narrow_passage/";
	//		std::string prmFolder = "/home/khan/phd_ws/traversability/prm/narrow_passage/";
	//		std::string adjFolder = "/home/khan/phd_ws/traversability/adjacency/narrow_passage/";
	//		std::string fileStartName = "narrow_";

	std::string pointcloudFolder = "/home/khan/phd_ws/traversability/pointclouds/synthetic/";
	std::string prmFolder = "/home/khan/phd_ws/traversability/prm/test/";
	std::string adjFolder = "/home/khan/phd_ws/traversability/adjacency/test/";
	std::string fileStartName = "road_long0";

	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir (pointcloudFolder.c_str())) != NULL) {
		while ((ent = readdir (dir)) != NULL) {
			std::string fName = ent->d_name;
			if(fName.find(fileStartName)!=std::string::npos)
			{
				bool fileProcessed = false;
				DIR *dirPRM;
				struct dirent *entPRM;
				if ((dirPRM = opendir (adjFolder.c_str())) != NULL) {
					while ((entPRM = readdir (dirPRM)) != NULL) {
						std::string fNamePRM = entPRM->d_name;
						if(fNamePRM.find(fileStartName)!=std::string::npos) {
							std::string fProcessed = fNamePRM.substr(0,fNamePRM.find("_prm_view.pcd"));
							std::string f2Process = fName.substr(0,fName.find(".pcd"));
							if (f2Process.compare(fProcessed)==0) {
								fileProcessed = true;
								std::cout <<"file processed already"<<f2Process<<"\t"<<fProcessed<<std::endl;
								break;
							}
						}
					}
				}

				if(fileProcessed) {
					continue;
				}

//				double cpu1  = get_cpu_time();
				std::cout << endl<<"reading file -------------------------------------\t"<<fName.c_str() << std::endl;
				std::stringstream readFile;
				readFile << pointcloudFolder.c_str() <<fName.substr(0,fName.find(".pcd")) << ".pcd";
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
				reader.read (readFile.str(), *cloud_in);

				std::cout << "Points in cloud before filtering: " << cloud_in->points.size() << std::endl;

				start = std::clock();

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

				// your test
				std::cout << "Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
				//				int config_count = prmObj.getConfigurationX() * prmObj.getConfigurationY() * prmObj.getConfigurationTheta(); 

				/*
				 * save adjacency
				 */
				std::stringstream saveAdj;
				saveAdj << adjFolder.c_str() << fName.substr(0,fName.find(".pcd")) <<"_adj";
				std::ofstream saveAdjFile(saveAdj.str().c_str());
				if (saveAdjFile.is_open())
				{
					for (int i=0; i < prm_graph.size(); i++) 
					{
						saveAdjFile <<prm_graph[i].first<<" "<<prm_graph[i].second<<" "<<1<< "\n";
					}
					saveAdjFile.close();
				}else {
					std::cout<<"Error: can not find directory"<<std::endl;
				}

				/*
				 * save configurations
				 */
				Eigen::MatrixXf vehicle_configs = prmObj.getConfigurations();
				std::vector<int> config_validity_status = prmObj.getConfigurationValidityStatus();

				std::stringstream savePRM;
				savePRM << prmFolder.c_str() << fName.substr(0,fName.find(".pcd")) <<"_prm";
				std::ofstream savePRMFile(savePRM.str().c_str());
				if (savePRMFile.is_open())
				{
					for (int i=0; i < config_validity_status.size(); i++) 
					{
						savePRMFile << vehicle_configs.col(i).transpose()<<" "<<config_validity_status[i]<< "\n";
					}
					savePRMFile.close();
				}else {
					std::cout<<"Error: can not find directory"<<std::endl;
				}
//				double cpu2  = get_cpu_time();
//				std::cout << "CPU Time after prm complete = " << cpu2  - cpu1  << std::endl;
				std::cout << "file written successfully" << endl<< std::endl;
			}
		}
		closedir (dir);
	} else {
		perror ("could not open directory");
		return EXIT_FAILURE;
	}
	return 0;
}

