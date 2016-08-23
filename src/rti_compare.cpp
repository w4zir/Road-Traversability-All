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

    std::string pointcloudFolder = "/home/mudassir/phd_ws/traversability/pointclouds/clearance_roads/";

	std::string fileStartName = ".pcd";

	for (int i_counter = 1; i_counter<=10; i_counter++)
	{
        std::stringstream adjFolderStream;
        adjFolderStream << "/home/mudassir/phd_ws/traversability/adjacency/clearance_roads/rand_01_" << i_counter << "/";
        std::string adjFolder = adjFolderStream.str().c_str();
        //adjFolder.append(std::to_string(i_counter));
       // adjFolder.append("/");
        std::stringstream configFolderStream;
        configFolderStream << "/home/mudassir/phd_ws/traversability/configs/clearance_roads/rand_01_" << i_counter << "/";
        std::string configFolder = configFolderStream.str().c_str();
         //configFolder.append(std::to_string(i_counter));
        //configFolder.append("/");
                std::cout << endl<<"reading nominal road file road_clear1.pcd -------------------------------------\t"<< std::endl;
				std::stringstream readBaseFile;
                readBaseFile << pointcloudFolder.c_str() <<"road_clear1.pcd";
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
                    /*
                     * reading obstacle file
                     */
                    Eigen::MatrixXf obstacles_info_tmp = Eigen::MatrixXf::Constant(1000,4,-1000);
                    Eigen::MatrixXf obstacles_info;
                    std::stringstream readObstaclesInfoFile;
                    readObstaclesInfoFile << pointcloudFolder.c_str() << fName.substr(0,fName.find(".pcd")) << "_obs";
                    std::cout <<"file name:\t" << readObstaclesInfoFile.str() << std::endl;
                    std::ifstream obsFile (readObstaclesInfoFile.str().c_str());
                    if (obsFile.is_open())
                    {

                        //cout<< "writing data to road_"<<fileIdx<<endl;
                        int c_id = 0;
                        while (!obsFile.eof())
                        {
                            std::string line;
                            std::getline (obsFile,line);
                            if(line.size()==0)
                            {
                                break;
                            }
                            std::size_t nospace1 = line.find_first_not_of(" ");
                            std::size_t space1 = line.find(" ",nospace1);
                            std::size_t nospace2 = line.find_first_not_of(" ",space1);
                            std::size_t space2 = line.find(" ",nospace2);
                            std::size_t nospace3 = line.find_first_not_of(" ",space2);
                            std::size_t space3 = line.find(" ",nospace3);
                            std::size_t nospace4 = line.find_first_not_of(" ",space3);
                            std::size_t space4 = line.find(" ",nospace4);

                            std::string xStr = line.substr(nospace1,space1-1);
                            std::string yStr = line.substr(nospace2,space2-1);
                            std::string rStr = line.substr(nospace3,space3-1);
                            std::string sStr = line.substr(nospace4,space4-1);
                            float xVal = std::atof(xStr.c_str());
                            float yVal = std::atof(yStr.c_str());
                            float rVal = std::atof(rStr.c_str());
                            float sVal = std::atof(sStr.c_str());
                           // std::cout <<c_id << ":\t line:	" << line <<"\t," << space1 <<"," << space2 <<"," << space3 << std::endl;

                            obstacles_info_tmp.row(c_id) = Eigen::Vector4f(xVal,yVal,rVal,sVal);
                            c_id++;
                            //std::cout <<c_id << ":\t" << xVal <<"," << yVal <<"," << rVal <<"," << sVal << std::endl;

                        }
                        obsFile.close();
                        obstacles_info = obstacles_info_tmp.topRows(c_id);
                    }
                    std::cout << obstacles_info.rows() <<" obstacles found.\n" << obstacles_info << std::endl;
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
                    prtObj.setObstaclesInfo(obstacles_info);
					prtObj.computePRT();
					std::vector< std::pair<int,int> > prt_graph;
					prtObj.getPRTAdjacencyList(prt_graph);

					std::cout << " prt_graph.size():" <<  prt_graph.size() << std::endl;

					std::cout << "Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl; 

					/*
					 * save adjacency
					 */
					std::stringstream saveAdj;
                    saveAdj << adjFolder.c_str() << fName.substr(0,fName.find(".pcd")) <<"_adj";
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
                        std::cout << adjFolder.c_str() << std::endl;
                         exit(0);
					}

					/*
					 * save configurations
					 */
					Eigen::MatrixXf vehicle_configs = prtObj.getConfigurations();
					std::vector<int> config_validity_status = prtObj.getConfigurationsValidityStatus();
					std::vector<float> config_clearance = prtObj.getConfigurationsClearance();

					std::stringstream saveConfigs;
                    saveConfigs << configFolder.c_str() << fName.substr(0,fName.find(".pcd")) <<"_conf";
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
                         exit(0);
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


