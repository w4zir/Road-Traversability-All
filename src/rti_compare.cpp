#include "include/prt.h"

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

	std::string pointcloudFolder = "/home/khan/phd_ws/traversability/pointclouds/safe_roads/";
	std::string adjFolder = "/home/khan/phd_ws/traversability/adjacency/safe_roads/rand/";
	std::string configFolder = "/home/khan/phd_ws/traversability/configs/safe_roads/rand/";
	std::string fileStartName = ".pcd";

	for (int i_counter = 1; i_counter<=10; i_counter++)
	{
				std::cout << endl<<"reading nominal road file road_long1.pcd -------------------------------------\t"<< std::endl;
				std::stringstream readBaseFile;
				readBaseFile << pointcloudFolder.c_str() <<"road_safe1.pcd";
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base (new pcl::PointCloud<pcl::PointXYZ>);
				reader.read (readBaseFile.str(), *cloud_base);
		
				std::cout << "Points in cloud before filtering: " << cloud_base->points.size() << std::endl;
		
				start = std::clock();
		
				// Create a set of planar coefficients with X=Y=0,Z=1
				pcl::ModelCoefficients::Ptr proj_plane_coefficients_base (new pcl::ModelCoefficients ());
				proj_plane_coefficients_base->values.resize (4);
				proj_plane_coefficients_base->values[0] = proj_plane_coefficients_base->values[1] = 0;
				proj_plane_coefficients_base->values[2] = 1.0;
				proj_plane_coefficients_base->values[3] = 0;
		
				pcl::PRT<pcl::PointXYZ> prtObjBase;
				prtObjBase.setInputCloud(cloud_base);
				prtObjBase.SetProjectedPlaneCoefficients(proj_plane_coefficients_base);
				prtObjBase.setImportConfigsFlag(false);
				prtObjBase.setRandomConfigsFlag(true);
				prtObjBase.computePRT();
				std::vector< std::pair<int,int> > prt_graph_base;
				prtObjBase.getPRTAdjacencyList(prt_graph_base);
		
				std::cout << "Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl; 
		
				/*
				 * save adjacency
				 */
//				std::stringstream saveAdjBase;
//				saveAdjBase << adjFolder.c_str() << "road_long0"  << "_" << i_counter <<"_adj";
//				std::ofstream saveAdjFileBase(saveAdjBase.str().c_str());
//				if (saveAdjFileBase.is_open())
//				{
//					for (int i=0; i < prt_graph_base.size(); i++) 
//					{
//						saveAdjFileBase << prt_graph_base[i].first <<" "<< prt_graph_base[i].second <<" "<<1<< "\n";
//					}
//					saveAdjFileBase.close();
//				}else {
//					std::cout<<"Error: can not find directory"<<std::endl;
//				}
		
				/*
				 * save configurations
				 */
				Eigen::MatrixXf vehicle_configs_import = prtObjBase.getConfigurations();
				std::vector<int> config_validity_status_base = prtObjBase.getConfigurationsValidityStatus();
				std::vector<float> config_clearance_base = prtObjBase.getConfigurationsClearance();
		
//				std::stringstream saveConfigsBase;
//				saveConfigsBase << PRTFolder.c_str() << "road_long0"  << "_" << i_counter <<"_prt";
//				std::ofstream saveConfigsFileBase(saveConfigsBase.str().c_str());
//				if (saveConfigsFileBase.is_open())
//				{
//					for (int i=0; i < vehicle_configs.rows(); i++) 
//					{
//						saveConfigsFileBase << vehicle_configs.row(i) <<" "<<config_validity_status_base[i]<< " " << config_clearance_base[i] << "\n";
//					}
//					saveConfigsFileBase.close();
//				}else {
//					std::cout<<"Error: can not find directory"<<std::endl;
//				}

		/*
		 * Reading Configuration file from folder
		 */
//		Eigen::MatrixXf vehicle_configs_import = Eigen::MatrixXf::Constant(7175,4,-1000);
//		std::stringstream readConfigFile;
//		readConfigFile << "/home/khan/phd_ws/traversability/configs/rti_compare2/1_0/road_long0_" << i_counter << "_conf";
//		//std::cout <<"file name:\t" << readConfigFile.str() << std::endl;
//		std::ifstream configFile (readConfigFile.str().c_str());
//		if (configFile.is_open())
//		{
//			
//			//cout<< "writing data to road_"<<fileIdx<<endl;
//			int c_id = 0;
//			while (!configFile.eof())
//			{
//				std::string line;
//				std::getline (configFile,line);
//				if(line.size()==0)
//											{
//												break;
//											}
//				std::size_t nospace1 = line.find_first_not_of(" ");
//				std::size_t space1 = line.find(" ",nospace1);
//				std::size_t nospace2 = line.find_first_not_of(" ",space1);
//				std::size_t space2 = line.find(" ",nospace2);
//				std::size_t nospace3 = line.find_first_not_of(" ",space2);
//				std::size_t space3 = line.find(" ",nospace3);
//
//				std::string xStr = line.substr(nospace1,space1-1);
//				std::string yStr = line.substr(nospace2,space2-1);
//				std::string tStr = line.substr(nospace3,space3-1);
//				float xVal = std::atof(xStr.c_str());
//				float yVal = std::atof(yStr.c_str());
//				float tVal = std::atof(tStr.c_str());
//				float pVal = 0;
//			//	std::cout <<c_id++ << ":\t line:	" << line <<"\t," << space1 <<"," << space2 <<"," << space3 << std::endl; 
//				
//				vehicle_configs_import.row(c_id) = Eigen::Vector4f(xVal,yVal,tVal,pVal);
//				c_id++;
//			//	std::cout <<c_id << ":\t" << xVal <<"," << yVal <<"," << tVal <<"," << pVal << std::endl; 
//				
//			}
//			configFile.close();
//		}

				/*
				 * apply it on all other files.
				 */
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
							//							std::cout <<"fNamePRM." << fNamePRM << std::endl;
							if(fNamePRM.find("_adj")!=std::string::npos) {
								std::stringstream fileNameee;
								fileNameee << "_" << i_counter <<"_adj";
								std::string fProcessed = fNamePRM.substr(0,fNamePRM.find(fileNameee.str()));
								std::string f2Process = fName.substr(0,fName.find(".pcd"));
								if (f2Process.compare(fProcessed)==0) {
									fileProcessed = true;
									std::cout <<"file processed already"<<f2Process<<"\t"<<fProcessed<<std::endl;
									break;
								}
							}
						}
					} else
					{
						std::cout <<"cant open directory." << std::endl;
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

					pcl::PRT<pcl::PointXYZ> prtObj;
					prtObj.setInputCloud(cloud_in);
					prtObj.SetProjectedPlaneCoefficients(proj_plane_coefficients);
					prtObj.setImportConfigsFlag(true);
					prtObj.setConfigurations(vehicle_configs_import);
					prtObj.computePRT();
					std::vector< std::pair<int,int> > prt_graph;
					prtObj.getPRTAdjacencyList(prt_graph);

					std::cout << " prt_graph.size():" <<  prt_graph.size() << std::endl;

					std::cout << "Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl; 

					/*
					 * save adjacency
					 */
					std::stringstream saveAdj;
					saveAdj << adjFolder.c_str() << fName.substr(0,fName.find(".pcd"))  << "_" << i_counter <<"_adj";
					std::ofstream saveAdjFile(saveAdj.str().c_str());
					if (saveAdjFile.is_open())
					{
						for (int i=0; i < prt_graph.size(); i++) 
						{
							saveAdjFile << prt_graph[i].first <<" "<< prt_graph[i].second <<" "<<1<< "\n";
						}
						saveAdjFile.close();
					}else {
						std::cout<<"Error: can not find directory"<<std::endl;
					}

					/*
					 * save configurations
					 */
					Eigen::MatrixXf vehicle_configs = prtObj.getConfigurations();
					std::vector<int> config_validity_status = prtObj.getConfigurationsValidityStatus();
					std::vector<float> config_clearance = prtObj.getConfigurationsClearance();

					std::stringstream saveConfigs;
					saveConfigs << configFolder.c_str() << fName.substr(0,fName.find(".pcd")) << "_" << i_counter <<"_conf";
					std::ofstream saveConfigsFile(saveConfigs.str().c_str());
					if (saveConfigsFile.is_open())
					{
						for (int i=0; i < vehicle_configs.rows(); i++) 
						{
							saveConfigsFile << vehicle_configs.row(i) <<" "<<config_validity_status[i]<< " " << config_clearance[i] << "\n";
						}
						saveConfigsFile.close();
					}else {
						std::cout<<"Error: can not find directory"<<std::endl;
					}
					std::cout << "file written successfully" << endl<< std::endl;
					//pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = prtObj.getDEMVisibilityCloud ();

				}
			}
			closedir (dir);
		} else {
			perror ("could not open directory");
			return EXIT_FAILURE;
		}
	}
	return 0;
}


