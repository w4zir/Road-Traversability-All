#include <string>
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

double const PI = 3.14159265;

int
main (int argc, char *argv[])
{
  pcl::PCDWriter writer;
  pcl::PCDReader reader;
  //  if(argc < 1)
  //  {
  //    std::cerr << argv[0] << "file_name" << std::endl;
  //  }
  //  std::string fileName = argv[1];
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //  if ( pcl::io::loadPCDFile <pcl::PointXYZ> (fileName.c_str(), *cloud) == -1)
  //  {
  //    std::cout << "Cloud reading failed." << std::endl;
  //    return (-1);
  //  }

  std::string dataFolder = "/home/mudassir/phd_ws/data/traversability/road_detection/2011_09_26_drive_0117_height/";
  std::string roadFolder = "/home/mudassir/phd_ws/data/traversability/road_detection/2011_09_26_drive_0117_height_normal_detect/";

  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir (dataFolder.c_str())) != NULL) {
    while ((ent = readdir (dir)) != NULL) {
      std::string fName = ent->d_name;
      if(fName.find("road_")!=std::string::npos)
      {
        bool fileProcessed = false;
        DIR *dirPRM;
        struct dirent *entPRM;
        if ((dirPRM = opendir (roadFolder.c_str())) != NULL) {
          while ((entPRM = readdir (dirPRM)) != NULL) {
            std::string fNamePRM = entPRM->d_name;
            if(fNamePRM.find("road_")!=std::string::npos) {
              std::string fProcessed = fNamePRM.substr(0,fNamePRM.find("_seg.pcd"));
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

        std::cout << endl<<"reading file -------------------------------------\t"<<fName.c_str() <<endl;
        std::stringstream readFile;
        // readFile << "/home/mudassir/phd_ws/data/pointclouds/test/"<<fileName.c_str() << ".pcd";
        readFile << dataFolder.c_str() <<fName.substr(0,fName.find(".pcd")) << ".pcd";
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        reader.read (readFile.str(), *cloud);

        pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod (tree);
        normal_estimator.setInputCloud (cloud);
        normal_estimator.setKSearch (50);
        normal_estimator.compute (*normals);

        //  pcl::IndicesPtr indices (new std::vector <int>);
        //  pcl::PassThrough<pcl::PointXYZ> pass;
        //  pass.setInputCloud (cloud);
        //  pass.setFilterFieldName ("z");
        //  pass.setFilterLimits (-2.0, 3.0);
        //  pass.filter (*indices);

        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize (500);
        reg.setMaxClusterSize (1000000);
        reg.setSearchMethod (tree);
        reg.setNumberOfNeighbours (40);
        reg.setInputCloud (cloud);
        //reg.setIndices (indices);
        reg.setInputNormals (normals);
        reg.setSmoothnessThreshold (6.0 / 180.0 * M_PI);
        reg.setCurvatureTestFlag(false);
       // reg.setCurvatureThreshold (2.0);

        std::vector <pcl::PointIndices> clusters;
        reg.extract (clusters);

        // std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
        // std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
        // std::cout << "These are the indices of the points of the initial" <<
        //    std::endl << "cloud that belong to the first cluster:" << std::endl;
        //  int counter = 0;
        //  while (counter < clusters.size ())
        //  {
        //    std::cout << "cluster size:\t" << clusters[counter].indices.size () << std::endl;
        //    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud
        //    counter++;
        //  }

        //  Eigen::Vector3f v1(0,0,1);
        //  int j = 0;
        //  for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it)
        //  {
        //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        //    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        //      cloud_cluster->points.push_back (cloud->points[*pit]); //*
        //    cloud_cluster->width = cloud_cluster->points.size ();
        //    cloud_cluster->height = 1;
        //    cloud_cluster->is_dense = true;
        //
        //    /************************************************************************************************************************************************************************
        //       * get planar coefficients
        //       * ***********************************************************************************************************************************************************************
        //       */
        //      pcl::SACSegmentation<pcl::PointXYZ> seg;
        //      pcl::ModelCoefficients::Ptr model_coeff (new pcl::ModelCoefficients);
        //      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        //      // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
        //      seg.setOptimizeCoefficients (true);
        //      seg.setModelType (pcl::SACMODEL_PLANE);
        //      seg.setMethodType (pcl::SAC_RANSAC);
        //      seg.setMaxIterations (100);
        //      seg.setDistanceThreshold (0.01);
        //      seg.setInputCloud (cloud_cluster);
        //      seg.segment (*inliers, *model_coeff);
        //      //std::cout << "coefficients are "<<*model_coeff<<std::endl;
        //      double coeffSqrt = std::sqrt(std::pow(model_coeff->values[0],2)+std::pow(model_coeff->values[1],2)+std::pow(model_coeff->values[2],2));
        //
        //      Eigen::Vector3f v2(model_coeff->values[0],model_coeff->values[1],model_coeff->values[2]);
        //      double angle = std::acos(v1.dot(v2))*180/PI;
        //      cout<<"angle:\t"<<angle << std::endl;
        //      /****************************
        //     * save pointcloud
        //     * *************************** */
        //
        //    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        //    std::stringstream ss;
        //    ss << "cloud_cluster_" << j << ".pcd";
        //    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
        //    j++;
        //  }
        //    std::cout << clusters[0].indices[counter] << ", ";
        //    counter++;
        //    if (counter % 10 == 0)
        //      std::cout << std::endl;
        pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
        std::cout << std::endl;

        std::stringstream saveRoadPCD;
        saveRoadPCD << roadFolder.c_str() << fName.substr(0,fName.find(".pcd")) << "_seg.pcd";
        writer.write<pcl::PointXYZRGB> (saveRoadPCD.str (), *colored_cloud, false);

        //  pcl::visualization::CloudViewer viewer ("Cluster viewer");
        //  viewer.showCloud(colored_cloud);
        //  while (!viewer.wasStopped ())
        //  {
        //  }
      }
    }
    closedir (dir);
  } else {
    perror ("could not open directory");
    return EXIT_FAILURE;
  }
  return (0);
}
