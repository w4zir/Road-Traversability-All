#include "include/prt.h"

#include <ctime>
#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>

double get_cpu_time(){
    return clock() / CLOCKS_PER_SEC;
}

int main(int argc, char *argv[])
{
    srand(time(0));
    pcl::PCDReader reader;
    pcl::PCDWriter writer;

    std::clock_t    start;


    std::string pointcloudFolder = "/home/mudassir/phd_ws/traversability/pointclouds/rsi/";
    std::string configFolder = "/home/mudassir/phd_ws/traversability/configs/rsi/rand_01/";
    std::string adjFolder = "/home/mudassir/phd_ws/traversability/adjacency/rsi/rand_01/";
    std::string fileStartName = ".pcd";

    //	std::string pointcloudFolder = "/home/wazir/phd_ws/traversability/pointclouds/tmp/";
    //	std::string configFolder = "/home/wazir/phd_ws/traversability/configs/tmp/";
    //	std::string adjFolder = "/home/wazir/phd_ws/traversability/adjacency/tmp/";
    //	std::string fileStartName = ".pcd";

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
                        if(fNamePRM.find("_adj")!=std::string::npos) {
                            std::string fProcessed = fNamePRM.substr(0,fNamePRM.find("_adj"));
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
                std::cout << endl<<"reading pointcloud file -------------------------------------\t"<<fName.c_str() << std::endl;
                std::stringstream readFile;
                readFile << pointcloudFolder.c_str() <<fName.substr(0,fName.find(".pcd")) << ".pcd";
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
                reader.read (readFile.str(), *cloud_in);

                std::cout << "Points in cloud before filtering: " << cloud_in->points.size() << std::endl;

                start = std::clock();

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

                // Create a set of planar coefficients with X=Y=0,Z=1
                pcl::ModelCoefficients::Ptr proj_plane_coefficients (new pcl::ModelCoefficients ());
                proj_plane_coefficients->values.resize (4);
                proj_plane_coefficients->values[0] = proj_plane_coefficients->values[1] = 0;
                proj_plane_coefficients->values[2] = 1.0;
                proj_plane_coefficients->values[3] = 0;

                pcl::PRT<pcl::PointXYZ> prtObj;
                prtObj.setInputCloud(cloud_in);
                prtObj.SetProjectedPlaneCoefficients(proj_plane_coefficients);
                prtObj.setRandomConfigsFlag(true);
                prtObj.setObstaclesInfo(obstacles_info);
                prtObj.computePRT();
                std::vector< std::pair<int,int> > prt_graph;
                prtObj.getPRTAdjacencyList(prt_graph);

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
                    exit(0);
                }

                /*
                 * save configurations
                 */
                Eigen::MatrixXf vehicle_configs = prtObj.getConfigurations();
                //				int config_counter = PRTObj.getConfigCounter();
                //				std::cout << "config counter:" << config_counter << std::endl;
                std::vector<int> config_validity_status = prtObj.getConfigurationsValidityStatus();
                std::vector<float> config_clearance = prtObj.getConfigurationsClearance();

                std::stringstream savePRT;
                savePRT << configFolder.c_str() << fName.substr(0,fName.find(".pcd")) <<"_conf";
                std::ofstream savePRTFile(savePRT.str().c_str());
                if (savePRTFile.is_open())
                {
                    for (int i=0; i < vehicle_configs.rows(); i++)
                    {
                        savePRTFile << vehicle_configs.row(i) <<" "<<config_validity_status[i]<< " " << config_clearance[i] << "\n";
                    }
                    savePRTFile.close();
                }else {
                    std::cout<<"Error: can not find directory"<<std::endl;
                    exit(0);
                }
                //				double cpu2  = get_cpu_time();
                //				std::cout << "CPU Time after prm complete = " << cpu2  - cpu1  << std::endl;
                std::cout << "file written successfully" << endl<< std::endl;

              //  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = prtObj.getDEMVisibilityCloud ();
                //  writer.write<pcl::PointXYZRGB> ("road_rgb.pcd", *colored_cloud, false);
                //				boost::shared_ptr<pcl::PolygonMesh> mesh_ptr_;
                //				mesh_ptr_->polygons[0].vertices[0] = 0;
                //				mesh_ptr_->polygons[0].vertices[1] = 0;
                //				mesh_ptr_->polygons[0].vertices[2] = 0;
                //				mesh_ptr_->polygons[1].vertices[0] = 1;
                //				mesh_ptr_->polygons[1].vertices[1] = 0;
                //				mesh_ptr_->polygons[1].vertices[2] = 0;
                //				mesh_ptr_->polygons[2].vertices[0] = 1;
                //				mesh_ptr_->polygons[2].vertices[1] = 2;
                //				mesh_ptr_->polygons[2].vertices[2] = 0;

                //				boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
                //				viewer->setBackgroundColor (0, 0, 0);
                //				pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colored_cloud);
                //				viewer->addPointCloud<pcl::PointXYZRGB> (colored_cloud, rgb, "sample cloud");
                //				viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
                //				viewer->addCoordinateSystem (1.0);
                //				viewer->initCameraParameters ();

                //				pcl::visualization::CloudViewer viewer ("Cluster viewer");

                //				viewer.addPolygonMesh	(mesh_ptr_,"polygon",0 );
                //				viewer.showCloud(colored_cloud);
                //				while (!viewer.wasStopped ())
                //				{
                //				}

                //				while (!viewer->wasStopped ())
                //				{
                //					viewer->spinOnce (100);
                //					boost::this_thread::sleep (boost::posix_time::microseconds (100000));
                //				}
            }
        }
        closedir (dir);
    } else {
        perror ("could not open directory");
        return EXIT_FAILURE;
    }
    return 0;
}


