#include "include/rrt.h"

#include <ctime>
#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

double get_cpu_time(){
	return clock() / CLOCKS_PER_SEC;
}

int main(int argc, char *argv[])
{
	srand(time(0));
	pcl::PCDReader reader;
	pcl::PCDWriter writer;

	std::clock_t    start;


	//		std::string pointcloudFolder = "/home/khan/phd_ws/traversability/pointclouds/narrow_passage/";
	//		std::string prmFolder = "/home/khan/phd_ws/traversability/prm/narrow_passage/";
	//		std::string adjFolder = "/home/khan/phd_ws/traversability/adjacency/narrow_passage/";
	//		std::string fileStartName = "narrow_";

	std::string pointcloudFolder = "/home/khan/phd_ws/traversability/pointclouds/long_roads/";
	std::string rrtFolder = "/home/khan/phd_ws/traversability/prm/long_roads/";
	std::string adjFolder = "/home/khan/phd_ws/traversability/adjacency/long_roads/";
	std::string fileStartName = ".pcd";

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

				pcl::RRT<pcl::PointXYZ> rrtObj;
				rrtObj.setInputCloud(cloud_in);
				rrtObj.SetProjectedPlaneCoefficients(proj_plane_coefficients);
				rrtObj.computeRRT();

				std::vector< std::pair<int,int> > rrt_graph;
				rrtObj.getRRTAdjacencyList(rrt_graph);

				std::cout << "Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl; 

				/*
				 * save adjacency
				 */
				std::stringstream saveAdj;
				saveAdj << adjFolder.c_str() << fName.substr(0,fName.find(".pcd")) <<"_adj";
				std::ofstream saveAdjFile(saveAdj.str().c_str());
				if (saveAdjFile.is_open())
				{
					for (int i=0; i < rrt_graph.size(); i++) 
					{
						saveAdjFile <<rrt_graph[i].first<<" "<<rrt_graph[i].second<<" "<<1<< "\n";
					}
					saveAdjFile.close();
				}else {
					std::cout<<"Error: can not find directory"<<std::endl;
				}

				/*
				 * save configurations
				 */
				Eigen::MatrixXf vehicle_configs = rrtObj.getConfigurations();
				int config_counter = rrtObj.getConfigCounter();
//				std::cout << "config counter:" << config_counter << std::endl;
			//	std::vector<int> config_validity_status = rrtObj.getConfigurationValidityStatus();

				std::stringstream saveRRT;
				saveRRT << rrtFolder.c_str() << fName.substr(0,fName.find(".pcd")) <<"_rrt";
				std::ofstream saveRRTFile(saveRRT.str().c_str());
				if (saveRRTFile.is_open())
				{
					for (int i=0; i < config_counter; i++) 
					{
						saveRRTFile << vehicle_configs.row(i)<< "\n";
					}
					saveRRTFile.close();
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

