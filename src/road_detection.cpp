/*
 * road_detection.cpp

 *
 *  Created on: Oct 5, 2015
 *      Author: mudassir
 */

/*
 * Detect road in a 3D pointcloud data
 */

#include <iostream>
#include <time.h>
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_search.h>
#include <pcl/visualization/cloud_viewer.h>

struct PointRoad
{
    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    float height;
    float probability;
    int roadType;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointRoad,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, height, height)
                                   (float, probability, probability)
                                   (int, roadType, roadType)
)

double const PI = 3.14159265;
double static search_radius = 0.2;
int const x_dim = 200;
int const y_dim = 200;
int const laser_min_radius = 4;
Eigen::MatrixXf road_height(x_dim,y_dim);

//#include <pcl/filters/extract_indices.h>
//#include <pcl/segmentation/progressive_morphological_filter.h>
//struct PointXF
//{
//    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
//    float height;
//    float gradient;
//    float gradient;
//    int valid;
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
//} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment
//
//POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZTP,           // here we assume a XYZ + "test" (as fields)
//                                   (float, x, x)
//                                   (float, y, y)
//                                   (float, z, z)
//                                   (float, theta, theta)
//                                   (float, phi, phi)
//                                   (int, valid, valid)
//)
pcl::PointCloud<pcl::PointXYZRGB> getGridMap(float x_min,float x_max,float y_min,float y_max,int x_grid_size, int y_grid_size,double laser_min_rad)
                                                                                                                                                                                                                                        {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_grid (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::cout << "enter function getVehicleCOnfigurations \n";

  float x_res = (x_max-x_min)/(x_grid_size-1); // because start and end are included
  float y_res = (y_max-y_min)/(y_grid_size-1);  // because start and end are included

  for (int i=0;i<=x_grid_size;i++) {
    for (int j=0;j<=y_grid_size;j++) {
      pcl::PointXYZRGB point;
      point.x = x_min + i*x_res;
      point.y = y_min + j*y_res;
      //point.x = xMin + wbWidth/2 + wheelWidth/2 + j*wInc;
      //point.y = yMin + wbLength/2 + wheelWidth/2 + i*lInc;
      point.z = 0.0f;
      double pos_radial_len = std::sqrt(std::pow(point.x,2)+std::pow(point.y,2));
      point.r = 0;
      if(pos_radial_len<laser_min_rad)
      {
        point.g = 255;
      } else
      {
        point.g = 0;
      }
      point.b = 0;
      cloud_grid->points.push_back(point);
    }
  }
  return *cloud_grid;
                                                                                                                                                                                                                                        }

pcl::PointCloud<PointRoad> getGridRoad(float x_min,float x_max,float y_min,float y_max,int x_grid_size, int y_grid_size,double laser_min_rad)
                                                                                                                                                                                                                                        {
  pcl::PointCloud<PointRoad>::Ptr cloud_grid (new pcl::PointCloud<PointRoad>);
  std::cout << "enter function getVehicleCOnfigurations \n";

  float x_res = (x_max-x_min)/(x_grid_size); // because start and end are included
  float y_res = (y_max-y_min)/(y_grid_size);  // because start and end are included

  for (int i=0;i<=x_grid_size;i++) {
    for (int j=0;j<=y_grid_size;j++) {
      PointRoad point;
      point.x = x_min + i*x_res;
      point.y = y_min + j*y_res;
      //point.x = xMin + wbWidth/2 + wheelWidth/2 + j*wInc;
      //point.y = yMin + wbLength/2 + wheelWidth/2 + i*lInc;
      point.z = 0.0f;
      point.height = 0;
      point.probability = 0;
      double pos_radial_len = std::sqrt(std::pow(point.x,2)+std::pow(point.y,2));
      if(pos_radial_len<laser_min_rad)
      {
        point.roadType = 1;
      } else
      {
        point.roadType = 0;
      }
      cloud_grid->points.push_back(point);
    }
  }
  return *cloud_grid;
                                                                                                                                                                                                                                        }
//pcl::PointCloud<pcl::PointXYZ>::Ptr projectCloudOnPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
//  pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::ProjectInliers<pcl::PointXYZ> proj;
//  pcl::ModelCoefficients::Ptr model_coeff;
//  model_coeff->values[0] = 0;
//  model_coeff->values[1] = 0;
//  model_coeff->values[2] = 1;
//  model_coeff->values[3] = 0;
//  proj.setModelType (pcl::SACMODEL_PLANE);
//  proj.setInputCloud (cloud);
//  proj.setModelCoefficients (model_coeff);
//  proj.filter (*projected_cloud);
//
//  std::cerr << "Cloud after project: " << std::endl;
//  std::cerr << *projected_cloud << std::endl;
//
//  return projected_cloud;
//}
//Eigen::VectorXd getNeighborProbabilities(int x_indx,int y_indx,int x_dim,int y_dim,Eigen::MatrixXf road_prob)
//{
//  Eigen::VectorXd n_probabilies;
//  for (int i=-1;i<=1;i++)
//  {
//    for (int j=-1;j<=1;j++)
//    {
//      int n_x = x_indx+i;
//      int n_y = y_indx+j;
//      if(n_x<0 || n_x>=x_dim || n_y<0 || n_y>=y_dim)
//      {
//         continue;
//      }else
//      {
//        n_probabilies
//      }
//
//    }
//  }
//}

int
main (int argc, char** argv)
{
  /* initialize random seed: */
  std::srand (std::time(NULL));

  std::string dataFolder = "/home/mudassir/phd_ws/data/traversability/pointclouds/kitti/2011_09_26_drive_0117_dem/";
  std::string roadFolder = "/home/mudassir/phd_ws/data/traversability/road_detection/temp/";

  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir (dataFolder.c_str())) != NULL) {
    while ((ent = readdir (dir)) != NULL) {
      std::string fName = ent->d_name;
      if(fName.find("road_60")!=std::string::npos)
      {
        bool fileProcessed = false;
        DIR *dirPRM;
        struct dirent *entPRM;
        if ((dirPRM = opendir (roadFolder.c_str())) != NULL) {
          while ((entPRM = readdir (dirPRM)) != NULL) {
            std::string fNamePRM = entPRM->d_name;
            if(fNamePRM.find("road_")!=std::string::npos) {
              std::string fProcessed = fNamePRM.substr(0,fNamePRM.find(".pcd"));
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
          //continue;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr grid_cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_project (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr grid_normals (new pcl::PointCloud<pcl::Normal>);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr planar_based_road (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr passthrough_based_road (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr dist_based_road (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr grid_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::PointCloud<PointRoad>::Ptr grid_road (new pcl::PointCloud<PointRoad>);
        pcl::PointCloud<PointRoad>::Ptr grid_road_projected (new pcl::PointCloud<PointRoad>);
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr planar_segmentation (new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::PCDWriter writer;
        //pcl::PointIndicesPtr ground (new pcl::PointIndices);

        // Fill in the cloud data
        pcl::PCDReader reader;
        //  reader.read ("road_0.pcd", *cloud_in);

        std::cout << endl<<"reading file -------------------------------------\t"<<fName.c_str() <<endl;
        std::stringstream readFile;
        //readFile << "/home/mudassir/phd_ws/data/pointclouds/test/"<<fileName.c_str() << ".pcd";
        readFile << dataFolder.c_str() <<fName.substr(0,fName.find(".pcd")) << ".pcd";
        //        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        reader.read (readFile.str(), *cloud_in);

        //  std::cerr << "Cloud before filtering: " << std::endl;
        //  std::cerr << *cloud_in << std::endl;

        //  road_height = Eigen::MatrixXf::Constant(x_dim,y_dim,0);

        /************************************************************************************************************************************************************************
         * get planar coefficients
         * ***********************************************************************************************************************************************************************
         */
        //  pcl::SACSegmentation<pcl::PointXYZ> seg;
        //  pcl::ModelCoefficients::Ptr model_coeff (new pcl::ModelCoefficients);
        //  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        //  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
        //  seg.setOptimizeCoefficients (true);
        //  seg.setModelType (pcl::SACMODEL_PLANE);
        //  seg.setMethodType (pcl::SAC_RANSAC);
        //  seg.setMaxIterations (100);
        //  seg.setDistanceThreshold (0.01);
        //  seg.setInputCloud (cloud_in);
        //  seg.segment (*inliers, *model_coeff);
        //  std::cout << "coefficients are "<<*model_coeff<<std::endl;
        //  double coeffSqrt = std::sqrt(std::pow(model_coeff->values[0],2)+std::pow(model_coeff->values[1],2)+std::pow(model_coeff->values[2],2));
        /* ***********************************************************************************************************************************************************************
         * Segment planar region with green color
         * *********************************************************************************************************************************************************************** */
        //  for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_in->points.begin (); it != cloud_in->points.end (); ++it)
        //  {
        //    pcl::PointXYZRGB point;
        //    point.x = it->x;
        //    point.y = it->y;
        //    point.z = it->z;
        //    point.r = 255;
        //    point.g = 0;
        //    point.b = 0;
        //    planar_based_road->push_back(point);
        //  }
        //  // dist_based_road = planar_based_road;
        //
        //  for (int i=0;i<inliers->indices.size();i++)
        //  {
        //    planar_based_road->points[i].r = 0;
        //    planar_based_road->points[i].g = 255;
        //  }
        //  writer.write<pcl::PointXYZRGB> ("road_planar.pcd", *planar_based_road, false);

        /************************************************************************************************************************************************************************
         * Segment road portion through passthrough filter
         * *********************************************************************************************************************************************************************** */
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough_road (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud_in);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (4.0, 6.0);
        pass.filter (*cloud_passthrough_road);
        pass.setInputCloud (cloud_passthrough_road);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (-2.0, 2.0);
        pass.filter (*cloud_passthrough_road);

        pcl::SACSegmentation<pcl::PointXYZ> seg_pass;
        pcl::ModelCoefficients::Ptr model_coeff_pass (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_pass (new pcl::PointIndices);
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
        seg_pass.setOptimizeCoefficients (true);
        seg_pass.setModelType (pcl::SACMODEL_PLANE);
        seg_pass.setMethodType (pcl::SAC_RANSAC);
        seg_pass.setMaxIterations (100);
        seg_pass.setDistanceThreshold (0.01);
        seg_pass.setInputCloud (cloud_passthrough_road);
        seg_pass.segment (*inliers_pass, *model_coeff_pass);
        std::cout << "coefficients are "<<*model_coeff_pass<<std::endl;
        double coeffSqrt_pass = std::sqrt(std::pow(model_coeff_pass->values[0],2)+std::pow(model_coeff_pass->values[1],2)+std::pow(model_coeff_pass->values[2],2));

        //  for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_in->points.begin (); it != cloud_in->points.end (); ++it)
        //  {
        //    double dist_from_plane = std::abs((model_coeff_pass->values[0]*it->x +
        //        model_coeff_pass->values[1]*it->y +
        //        model_coeff_pass->values[2]*it->z + model_coeff_pass->values[3]))/
        //            (coeffSqrt_pass);
        //    // cout <<"cloud_passthrough_road point\t"<<it->x<<","<<it->y<<","<<it->z<< "\t distance:\t"<<dist_from_plane<<std::endl;
        //    pcl::PointXYZRGB point;
        //    point.x = it->x;
        //    point.y = it->y;
        //    point.z = it->z;
        //    point.r = 0;
        //    point.g = 0;
        //    point.b = 0;
        //    if(dist_from_plane < 0.1)
        //    {
        //      point.g = 255;
        //    } else {
        //      point.r = 255;
        //    }
        //    passthrough_based_road->push_back(point);
        //  }
        //  writer.write<pcl::PointXYZRGB> ("road_passthrough.pcd", *passthrough_based_road, false);

        /************************************************************************************************************************************************************************
         * ************************************* project pointcloud on the road plane *********************************************************************************
         * *********************************************************************************************************************************************************************** */
        pcl::ProjectInliers<pcl::PointXYZ> proj; // (new pcl::ProjectInliers<pcl::PointXYZ>);
        proj.setModelType (pcl::SACMODEL_PLANE);
        proj.setInputCloud (cloud_in);
        proj.setModelCoefficients (model_coeff_pass);
        proj.filter (*cloud_project);
        std::cerr << "Cloud after project: " << std::endl;
        std::cerr << *cloud_project << std::endl;

        /*******************************************************************************************************************************
         **************************** Find average height of each point in pointcloud using its neighbors **************************
         *******************************************************************************************************************************/
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        tree->setInputCloud(cloud_project);

        pcl::PointCloud<pcl::PointXYZ>::Ptr height_cloud (new pcl::PointCloud<pcl::PointXYZ>);

        for (int it=0;it<cloud_project->points.size();it++)
        {
          double height_sum = 0;
          pcl::PointXYZ point;
          point.x = cloud_project->points[it].x;
          point.y = cloud_project->points[it].y;
          point.z = cloud_project->points[it].z;
          std::vector<int> pointIdx;
          std::vector<float> pointDist;
          tree->radiusSearch(point,search_radius,pointIdx,pointDist,0);

          if(pointIdx.size()!=0)
          {
            for (int i=0;i<pointIdx.size();i++)
            {
              double dist_from_road = std::abs((model_coeff_pass->values[0]*cloud_in->points[pointIdx[i]].x +
                  model_coeff_pass->values[1]*cloud_in->points[pointIdx[i]].y +
                  model_coeff_pass->values[2]*cloud_in->points[pointIdx[i]].z + model_coeff_pass->values[3]))/
                      (coeffSqrt_pass);
              height_sum += dist_from_road;
            }
            double avg_height = height_sum/pointIdx.size();
            pcl::PointXYZ point_to_add;
            point_to_add.x = cloud_project->points[it].x + model_coeff_pass->values[0]*avg_height;
            point_to_add.y = cloud_project->points[it].y + model_coeff_pass->values[1]*avg_height;
            point_to_add.z = cloud_project->points[it].z + model_coeff_pass->values[2]*avg_height;

            height_cloud->points.push_back(point_to_add);
            //grid_road_projected->points[it].height = avg_height;
          }else
          {
            pcl::PointXYZ point_to_add;
            point_to_add.x = cloud_project->points[it].x;
            point_to_add.y = cloud_project->points[it].y;
            point_to_add.z = cloud_project->points[it].z;

            height_cloud->points.push_back(point_to_add);
          }
          height_cloud->height = 1;
          height_cloud->width = height_cloud->points.size();
        }

        /************************************************************************************************************************************************************************
         * ************************************* Segment cloud using normal vectors *********************************************************************************
         * *********************************************************************************************************************************************************************** */
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr road_seg (new pcl::PointCloud<pcl::PointXYZRGB>);
        /*********************************************************
         * ******** Calculating Normals ***********************
         *******************************************************/
        pcl::search::Search<pcl::PointXYZ>::Ptr treeNorm = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud (cloud_in);
        ne.setSearchMethod (treeNorm);
        ne.setKSearch (50);
        ne.compute (*normals);

        //        /*********************************************************
        //         * ******** Segmentation using normals ***********************
        //         *******************************************************/
        //        Eigen::Vector3f v1(model_coeff_pass->values[0],model_coeff_pass->values[1],model_coeff_pass->values[2]);
        //        int counter0 = 0;
        //        int counter1 = 0;
        //        for (int it=0;it<height_cloud->points.size();it++)
        //        {
        //          Eigen::Vector3f v2(normals->points[it].normal_x,normals->points[it].normal_y,normals->points[it].normal_z);
        //          double angle = std::acos(v1.dot(v2))*180/PI;
        //          // cout<<"angle:\t"<< angle <<std::endl;
        //          pcl::PointXYZRGB point;
        //          point.x = height_cloud->points[it].x;
        //          point.y = height_cloud->points[it].y;
        //          point.z = height_cloud->points[it].z;
        //          point.r = 0;
        //          point.g = 0;
        //          point.b = 0;
        //
        //          if(angle<5)
        //          {
        //            counter0++;
        //            point.g = 255;
        //          }else
        //          {
        //            counter1++;
        //            point.r = 255;
        //          }
        //          road_seg->push_back(point);
        //        }
        //        cout<<"road points:\t"<<counter0 << "\t and non road:\t" <<counter1<< std::endl;
        //
        //        /***************************************************************************************
        //         * ******** Post process segmented cloud using planar surronding  ***********************
        //         ***************************************************************************************/        
        //        //        for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it = road_seg->points.begin (); it != road_seg->points.end (); ++it)
        //        //        {
        //        //          double dist_from_plane = std::abs((model_coeff_pass->values[0]*it->x +
        //        //              model_coeff_pass->values[1]*it->y +
        //        //              model_coeff_pass->values[2]*it->z + model_coeff_pass->values[3]))/
        //        //                  (coeffSqrt_pass);
        //        //          if(dist_from_plane <=0.1 && it->r ==255)
        //        //          {
        //        //            it->r = 0;
        //        //            it->g = 255;
        //        //          }
        //        //          if(dist_from_plane >0.1 && it->g ==255)
        //        //          {
        //        //            it->r = 255;
        //        //            it->g = 0;
        //        //          }
        //        //        }
        //        std::vector<int> indices_list;
        //        indices_list.resize(cloud_project->points.size(),0);
        //        std::vector<int> processed_list;
        //        processed_list.resize(cloud_project->points.size()+1,-1);
        //        for(int i=0;i<cloud_project->points.size();i++)
        //        {
        //          indices_list[i]=i;
        //        }
        //
        //        std::vector<double> exec_time;
        //        exec_time.resize(4,0);
        //        double t1,t2,t3,t4,t5;
        //        int it=0;
        //        int total_processed = 0;
        //        while(it < cloud_project->points.size())
        ////        for (int it=0; it<cloud_project->points.size(); it++)
        //        {
        //         t1 = clock() / CLOCKS_PER_SEC;
        //
        //          pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        //          pcl::PointXYZ point;
        //          point.x = cloud_project->points[indices_list[it]].x;
        //          point.y = cloud_project->points[indices_list[it]].y;
        //          point.z = cloud_project->points[indices_list[it]].z;
        //          std::vector<int> pointIdx;
        //          std::vector<float> pointDist;
        //          tree->radiusSearch(point,0.5,pointIdx,pointDist,0);
        //          t2 = clock() / CLOCKS_PER_SEC;      
        //                      exec_time[0] += t2-t1;
        //          //  std::cout <<"local neighbor points:" << pointIdx.size() << std::endl;
        //          if(pointIdx.size()>50)
        //          {
        //            t1 = clock() / CLOCKS_PER_SEC;
        //            for (int i_point=0;i_point<pointIdx.size();i_point++)
        //            {
        //              //              if(road_seg->points[pointIdx[i_point]].g==255)
        //              //              {
        //              pcl::PointXYZ point_to_add;
        //              point_to_add.x = road_seg->points[pointIdx[i_point]].x;
        //              point_to_add.y = road_seg->points[pointIdx[i_point]].y;
        //              point_to_add.z = road_seg->points[pointIdx[i_point]].z;
        //              local_cloud->push_back(point_to_add);
        //              //              }
        //            }
        //            processed_list[it] = 1;
        //            total_processed++;
        //            t3 = clock() / CLOCKS_PER_SEC; 
        //            exec_time[1] += t3-t1;
        //            //    std::cout <<"local valid neighbor points:" << local_cloud->points.size() << std::endl;
        //            pcl::SACSegmentation<pcl::PointXYZ> local_seg;
        //            pcl::ModelCoefficients::Ptr local_coeff (new pcl::ModelCoefficients);
        //            pcl::PointIndices::Ptr local_inlier (new pcl::PointIndices);
        //            local_seg.setOptimizeCoefficients (true);
        //            local_seg.setModelType (pcl::SACMODEL_PLANE);
        //            local_seg.setMethodType (pcl::SAC_RANSAC);
        //            local_seg.setMaxIterations (100);
        //            local_seg.setDistanceThreshold (0.01);
        //            local_seg.setInputCloud (local_cloud);
        //            local_seg.segment (*local_inlier, *local_coeff);
        //            t4 = clock() / CLOCKS_PER_SEC; 
        //            exec_time[2] += t4-t3;
        //            
        //            if(local_inlier->indices.size()>0)
        //            {    
        //              double loca_coeff_sqrt = std::sqrt(std::pow(local_coeff->values[0],2)+std::pow(local_coeff->values[1],2)+std::pow(local_coeff->values[2],2));
        //
        //              Eigen::Vector3f v2(local_coeff->values[0],local_coeff->values[1],local_coeff->values[2]);
        //              double angle = std::acos(v1.dot(v2))*180/PI;
        //              if(angle<5)
        //              {
        //
        //                std::vector<int> neighborIdx;
        //                std::vector<float> neighborDist;
        //                tree->radiusSearch(point,0.3,neighborIdx,neighborDist,0);
        //                std::cout <<"processing neibors:" << neighborIdx.size() << std::endl;
        //
        //                for(int inl_point=0; inl_point< neighborIdx.size(); inl_point++)
        //                {
        //                  processed_list[neighborIdx[inl_point]]=1;
        //
        //                  double dist_from_plane = std::abs((local_coeff->values[0]*road_seg->points[neighborIdx[inl_point]].x +
        //                      local_coeff->values[1]*road_seg->points[neighborIdx[inl_point]].y +
        //                      local_coeff->values[2]*road_seg->points[neighborIdx[inl_point]].z + local_coeff->values[3]))/
        //                          (loca_coeff_sqrt);
        //
        //                  if(dist_from_plane <=0.1 && road_seg->points[neighborIdx[inl_point]].r ==255)
        //                  {
        //                    road_seg->points[neighborIdx[inl_point]].r = 0;
        //                    road_seg->points[neighborIdx[inl_point]].g = 255;
        //                  }
        //                  if(dist_from_plane >0.1 && road_seg->points[neighborIdx[inl_point]].g ==255)
        //                  {
        //                    road_seg->points[neighborIdx[inl_point]].r = 255;
        //                    road_seg->points[neighborIdx[inl_point]].g = 0;
        //                  }
        //                }
        //                total_processed +=  neighborIdx.size();
        //              }      
        //            }else
        //            {
        //              std::vector<int> neighborIdx;
        //              std::vector<float> neighborDist;
        //              tree->radiusSearch(point,0.3,neighborIdx,neighborDist,0);
        //              std::cout <<"processing neibors:" << neighborIdx.size() << std::endl;
        //
        //              for(int inl_point=0; inl_point< neighborIdx.size(); inl_point++)
        //              {
        //                processed_list[neighborIdx[inl_point]]=1;
        //              }
        //            }
        //            // cout<<"angle:\t"<< angle <<std::endl;
        //            //            if (local_inlier->indices.size () != 0)
        //            //            {
        //            //              float road_points_ratio =  static_cast<float>(local_inlier->indices.size ()) / static_cast<float>(local_cloud->points.size());
        //            //              std::cout << "inliers ratio:" << road_points_ratio << std::endl;
        //            //            }
        //          }else
        //          {
        //            processed_list[it] = 1;
        //            total_processed++;
        //          }
        //
        //          for(int i=0; i < cloud_project->points.size()+1000;i++)
        //          {
        //            if(processed_list[i] == -1)
        //            {
        //              it=i;
        //              break;
        //            }
        //          }
        //          t5 = clock() / CLOCKS_PER_SEC; 
        //          exec_time[3] += t5-t1;
        //          
        //          
        ////          exec_time[3] += t4-t1;
        ////          std::cout << "points processed:" << total_processed << "\t pointer at:" << it << std::endl;
        //        } 
        //       std::cout << "part1:" << exec_time[0] << "\t part2:" << exec_time[1] << "\t part3:" << exec_time[2] << "\ttotal time:" << exec_time[3] << std::endl;

        //        pcl::visualization::CloudViewer viewer ("Cluster viewer");
        //        viewer.showCloud(road_seg);
        //        while (!viewer.wasStopped ())
        //        {
        //        }
        //
        //        writer.write<pcl::PointXYZRGB> ("road_detect.pcd", *road_seg, false);
        //
        //        return 0;

        reader.read ("road_detect.pcd", *road_seg);
        /************************************************************************************************************************************************************************
         * ************************************* Get rotation from xy-plane to road plane *********************************************************************************
         * *********************************************************************************************************************************************************************** */
        Eigen::Vector3f normalXYPlane (0.0,0.0,1.0);
        Eigen::Vector3f normalRoadPlane (model_coeff_pass->values[0],model_coeff_pass->values[1],model_coeff_pass->values[2]);
        //      Eigen::Vector3f normalRoadPlane (0.0,0.0,1.0);
        //      Eigen::Vector3f normalXYPlane (model_coeff_pass->values[0],model_coeff_pass->values[1],model_coeff_pass->values[2]);
        Eigen::Vector3f cNormal = normalXYPlane.cross(normalRoadPlane);
        float sNormal = sqrt(cNormal.squaredNorm());
        float dNormal = normalXYPlane.dot(normalRoadPlane);
        Eigen::Matrix3f cNormalSkewSymm;
        cNormalSkewSymm << 0, -cNormal(2), cNormal(1), cNormal(2), 0, -cNormal(0), -cNormal(1), cNormal(0), 0;
        Eigen::Matrix3f rot_from_xy;
        rot_from_xy = Eigen::Matrix3f::Identity(3,3)+cNormalSkewSymm+cNormalSkewSymm*cNormalSkewSymm*((1-dNormal)/pow(sNormal,2));
        //  std::cout<<"matrix:" << rot_from_xy << " inverse:" <<rot_from_xy.inverse() << std::endl;

        /*********************************************************
         * ******** Transform pointcloud to xy-plane ***********************
         *******************************************************/
        pcl::PointCloud<pcl::PointXYZ>::Ptr grid_xyz (new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        double const dist_of_plane_origin = model_coeff_pass->values[3]/(coeffSqrt_pass);
        transform.translation() << -dist_of_plane_origin*model_coeff_pass->values[0], -dist_of_plane_origin*model_coeff_pass->values[1], -dist_of_plane_origin*model_coeff_pass->values[2];
        transform.rotate (rot_from_xy.inverse());
        pcl::transformPointCloud (*cloud_project, *grid_xyz, transform);
        std::cout <<"points in transformed pointcloud:\t"<<grid_xyz->points.size()<<std::endl;
        /************************************************************************************************************************************************************************
         ************************************** Circular points search using direction vector ***********************************************************************************
         * *********************************************************************************************************************************************************************** */
        //        *grid_road = getGridRoad(-20,20,-20,20,x_dim,y_dim,laser_min_radius);
        //          grid_road->width = grid_road->points.size();
        //          grid_road->height = 1;
        //
        //        pcl::PointCloud<pcl::PointXYZ>::Ptr grid_xyz (new pcl::PointCloud<pcl::PointXYZ>);
        //        copyPointCloud(*grid_road,*grid_xyz);

        //        for (int j=0;j<grid_xyz->points.size();j++)
        //        {
        //          std::cout<<grid_xyz->points[j] << std::endl;
        //        }

        float octree_resolution = 0.1f;
        //        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_road (octree_resolution);// (new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>);
        ////        octree_road->setResolution(octree_resolution);
        //        octree_road.setInputCloud (cloud_project);
        //
        //        // octree_road.defineBoundingBox(-20,-20,-5,20,20,5);
        //        octree_road.addPointsFromInputCloud ();
        //        
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr octree_road  (new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(octree_resolution));
        //  octree_road->setResolution(octree_resolution);
        octree_road->setInputCloud (cloud_project);
        octree_road->addPointsFromInputCloud ();
        // std::cout<<"octree has point branches:\t"<< octree_road.getBranchCount()<< std::endl;

        Eigen::Vector3f origin_point (0.0,0.0,0.0);
        Eigen::Vector3f origin_trans;
        //     pcl::PointXYZ origin_p(0.0,0.0,-0.02);
        //     pcl::PointXYZ origin_trans;
        pcl::transformPoint(origin_point, origin_trans, transform);
        //  std::cout<<"transformed point:" << origin_trans << std::endl;
        double dist_from_road = std::abs((model_coeff_pass->values[0]*origin_trans[0] +
            model_coeff_pass->values[1]*origin_trans[1] +
            model_coeff_pass->values[2]*origin_trans[2] + model_coeff_pass->values[3]))/
                (coeffSqrt_pass);
        // std::cout<<"distance of transformed point from plane:" << dist_from_road << std::endl;
        //Eigen::Vector3f origin_point(origin_trans.x,origin_trans.y,origin_trans.z);
        //  std::cout <<"octree box:\t"<<octree_road.getBoundingBox() <<std::endl;

        //Eigen::Vector3f origin_point (the_point.x,the_point.y,the_point.z);
        Eigen::Vector3f plane_normal(model_coeff_pass->values[0],model_coeff_pass->values[1],model_coeff_pass->values[2]);
        //   double const dist_of_plane_origin = model_coeff_pass->values[3]/(coeffSqrt_pass);
        Eigen::Vector3f translation(-dist_of_plane_origin*model_coeff_pass->values[0], -dist_of_plane_origin*model_coeff_pass->values[1], -dist_of_plane_origin*model_coeff_pass->values[2]);


        std::vector<int> road_right;
                for (double theta=0;theta>=-180;theta-=1)
                {
                  Eigen::Vector3f xy_vector(cos(theta*PI/180.0), sin(theta*PI/180.0),0);
                  Eigen::Vector3f ray_direction;//(0.999941,0,0.01085);
                  ray_direction = rot_from_xy*xy_vector;
                  double dot_product = ray_direction.dot(plane_normal);
                  std::vector<int> pointIdx;
                  std::vector<float> dist;
                  octree_road->getIntersectedVoxelIndices(origin_trans,ray_direction,pointIdx,0);
                //  std::cout << "points in intersected ray:" << pointIdx.size() << std::endl;
                  int rCount = 0;
                  int gCount = 0;
                  double road_start_dist = -1;
                  double road_end_dist = -1;
                  double obs_start_dist = -1;
                  double obs_end_dist = -1;
                  for (int i = 0; i < pointIdx.size (); ++i)
                  {
                    double dist_from_origin = std::sqrt(std::pow(origin_trans[0]-cloud_project->points[pointIdx[i]].x,2)+std::pow(origin_trans[1]-cloud_project->points[pointIdx[i]].y,2)+std::pow(origin_trans[2]-cloud_project->points[pointIdx[i]].z,2));
                    //     std::cout << "distance from origin:" << dist_from_origin << std::endl;
                    if(dist_from_origin < 4)
                    {
                      continue;
                    }
                    if(road_seg->points[pointIdx[i]].r==255)
                    {
                      rCount++;
                      if(obs_start_dist==-1)
                      {
                        obs_start_dist = dist_from_origin;
                      }else
                      {
                        obs_end_dist = dist_from_origin;
                      }
                      // obstacle encountered, break out of loop
                      if((obs_end_dist-obs_start_dist)>0.2)
                      {
                        break;
                      }
                    }else if(road_seg->points[pointIdx[i]].g==255)
                    {
                      obs_start_dist = -1;
                      obs_end_dist = -1;
                      gCount++;
                      if(road_start_dist==-1)
                      {
                        road_start_dist = dist_from_origin;
                      }else
                      {
                        road_end_dist = dist_from_origin;
                      }
                      // obstacle encountered, break out of loop
                      if((road_end_dist-road_start_dist)>8)
                      {
                        road_right.clear();
                        for(int k=0;k<=i;k++)
                        {
                          road_right.push_back(pointIdx[k]);
                          road_seg->points[pointIdx[k]].r = 0;
                          road_seg->points[pointIdx[k]].g = 0;
                          road_seg->points[pointIdx[k]].b = 255;
                        }
                        //         std::cout<<"right side updated." << std::endl;
                        break;
                      }
                    }
                  }
                }

//        std::vector<int> road_left;
//        for (double theta=1;theta<=180;theta+=1)
//        {
//          Eigen::Vector3f xy_vector(cos(theta*PI/180.0), sin(theta*PI/180.0),0);
//          Eigen::Vector3f ray_direction;//(0.999941,0,0.01085);
//          ray_direction = rot_from_xy*xy_vector;
//          double dot_product = ray_direction.dot(plane_normal);
//          //   std::cout<<"dot product:" << dot_product << std::endl;
//          //    std::vector< pcl::PointXYZ, Eigen::aligned_allocator< pcl::PointXYZ > >  pointTIdx;
//          std::vector<int> pointIdx;
//          std::vector<float> dist;
//          // octree_road.getIntersectedVoxelCenters(origin_point,xy_vector,pointTIdx,0);
//          octree_road->getIntersectedVoxelIndices(origin_trans,ray_direction,pointIdx,0);
//          //  octree_road.radiusSearch(pcl::PointXYZ(5,0,0),2.0,pointIdx,dist,0);
//          //    std::cout<<"Points\t:"<<pointIdx.size()<<std::endl;
//
//          int rCount = 0;
//          int gCount = 0;
//          double road_start_dist = -1;
//          double road_end_dist = -1;
//          double obs_start_dist = -1;
//          double obs_end_dist = -1;
//          for (int i = 0; i < pointIdx.size (); ++i)
//          {
//            double dist_from_origin = std::sqrt(std::pow(origin_trans[0]-cloud_project->points[pointIdx[i]].x,2)+std::pow(origin_trans[1]-cloud_project->points[pointIdx[i]].y,2)+std::pow(origin_trans[2]-cloud_project->points[pointIdx[i]].z,2));
//            //     std::cout << "distance from origin:" << dist_from_origin << std::endl;
//            if(dist_from_origin < 4)
//            {
//              continue;
//            }
//            if(road_seg->points[pointIdx[i]].r==255)
//            {
//              rCount++;
//              if(obs_start_dist==-1)
//              {
//                obs_start_dist = dist_from_origin;
//              }else
//              {
//                obs_end_dist = dist_from_origin;
//              }
//              // obstacle encountered, break out of loop
//              if((obs_end_dist-obs_start_dist)>0.5)
//              {
//                break;
//              }
//            }else if(road_seg->points[pointIdx[i]].g==255)
//            {
//              gCount++;
//              obs_start_dist = -1;
//              obs_end_dist = -1;
//              if(road_start_dist==-1)
//              {
//                road_start_dist = dist_from_origin;
//              }else
//              {
//                road_end_dist = dist_from_origin;
//              }
//              // obstacle encountered, break out of loop
//              if((road_end_dist-road_start_dist)>5)
//              {
//                road_left.clear();
//                for(int k=0;k<=i;k++)
//                {
//                  road_left.push_back(pointIdx[k]);
//                  road_seg->points[pointIdx[k]].r = 0;
//                  road_seg->points[pointIdx[k]].g = 0;
//                  road_seg->points[pointIdx[k]].b = 255;
//                }
//                //     std::cout<<"left side updated." << std::endl;
//                break;
//              }
//            }
//
//          }
//        }

        pcl::PointXYZRGB origin_rgb;
        origin_rgb.x = origin_trans[0];
        origin_rgb.y = origin_trans[1];
        origin_rgb.z = origin_trans[2];
        origin_rgb.r = 0;
        origin_rgb.g = 0;
        origin_rgb.b = 255;

        road_seg->push_back(origin_rgb);
        //        writer.write<pcl::PointXYZRGB> ("road_line.pcd", *road_seg, false);

        //        writer.write<pcl::PointXYZ> ("road_height.pcd", *height_cloud, false);

      //  std::stringstream saveRoadPCD;
      //  saveRoadPCD << roadFolder.c_str() << fName.substr(0,fName.find(".pcd")) << ".pcd";
      //  writer.write<pcl::PointXYZRGB> (saveRoadPCD.str (), *road_seg, false);
        writer.write<pcl::PointXYZRGB> ("road_segmented.pcd", *road_seg, false);
      }
    }
    closedir (dir);
  } else {
    perror ("could not open directory");
    return EXIT_FAILURE;
  }

  /************************************************************************************************************************************************************************
   * ************************************* get grid map and project it on the road plane *********************************************************************************
   * *********************************************************************************************************************************************************************** */
  //  *grid_road = getGridRoad(-20,20,-20,20,x_dim,y_dim,laser_min_radius);
  //  grid_road->width = grid_road->points.size();
  //  grid_road->height = 1;
  //
  //  Eigen::Vector3f normalXYPlane (0.0,0.0,1.0);
  //  Eigen::Vector3f normalRoadPlane (model_coeff_pass->values[0],model_coeff_pass->values[1],model_coeff_pass->values[2]);
  //  Eigen::Vector3f cNormal = normalXYPlane.cross(normalRoadPlane);
  //  float sNormal = sqrt(cNormal.squaredNorm());
  //  float dNormal = normalXYPlane.dot(normalRoadPlane);
  //  Eigen::Matrix3f cNormalSkewSymm;
  //  cNormalSkewSymm << 0, -cNormal(2), cNormal(1), cNormal(2), 0, -cNormal(0), -cNormal(1), cNormal(0), 0;
  //  Eigen::Matrix3f rot2XYPlane;
  //  rot2XYPlane = Eigen::Matrix3f::Identity(3,3)+cNormalSkewSymm+cNormalSkewSymm*cNormalSkewSymm*((1-dNormal)/pow(sNormal,2));
  //  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  //  double const dist_of_plane_origin = model_coeff_pass->values[3]/(coeffSqrt_pass);
  //  transform.translation() << -dist_of_plane_origin*model_coeff_pass->values[0], -dist_of_plane_origin*model_coeff_pass->values[1], -dist_of_plane_origin*model_coeff_pass->values[2];
  //  transform.rotate (rot2XYPlane);
  //  pcl::transformPointCloud (*grid_road, *grid_road_projected, transform);
  //  std::cout <<"points in transformed pointcloud:\t"<<grid_road_projected->points.size()<<std::endl;

  /************************************************************************************************************************************************************************
   * ************************************* Find average distance of points in each grid cell from road plane ***************************************************************
   * *********************************************************************************************************************************************************************** */
  /***************************************************************************
   * ********  First project point cloud on road plane ***********************
   ***************************************************************************/
  //  pcl::ProjectInliers<pcl::PointXYZ> proj; // (new pcl::ProjectInliers<pcl::PointXYZ>);
  //  proj.setModelType (pcl::SACMODEL_PLANE);
  //  proj.setInputCloud (cloud_in);
  //  proj.setModelCoefficients (model_coeff_pass);
  //  proj.filter (*cloud_project);
  //  std::cerr << "Cloud after project: " << std::endl;
  //  std::cerr << *cloud_project << std::endl;

  /***************************************************************************
   **************************** Find average height **************************
   ***************************************************************************/
  //  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  //  tree->setInputCloud(cloud_project);
  //
  //  for (int it=0;it<grid_road_projected->points.size();it++)
  //  {
  //    double height_sum = 0;
  //    pcl::PointXYZ point;
  //    point.x = grid_road_projected->points[it].x;
  //    point.y = grid_road_projected->points[it].y;
  //    point.z = grid_road_projected->points[it].z;
  //    std::vector<int> pointIdx;
  //    std::vector<float> pointDist;
  //    tree->radiusSearch(point,search_radius,pointIdx,pointDist,0);
  //
  //    pcl::PointCloud<pcl::PointXYZ>::Ptr n_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //    if(pointIdx.size()!=0)
  //    {
  //      for (int i=0;i<pointIdx.size();i++)
  //      {
  //        double dist_from_road = std::abs((model_coeff_pass->values[0]*cloud_in->points[pointIdx[i]].x +
  //            model_coeff_pass->values[1]*cloud_in->points[pointIdx[i]].y +
  //            model_coeff_pass->values[2]*cloud_in->points[pointIdx[i]].z + model_coeff_pass->values[3]))/
  //                (coeffSqrt_pass);
  //        height_sum += dist_from_road;
  //      }
  //      double avg_height = height_sum/pointIdx.size();
  //      grid_road_projected->points[it].height = avg_height;
  //    }
  //  }

  /************************************************************************************************************************************************************************
   ***************************************************   get XYZ of grid pointcloud   **************************************************************************
   * *********************************************************************************************************************************************************************** */
  //  pcl::PointCloud<pcl::PointXYZ>::Ptr grid_xyz_projected (new pcl::PointCloud<pcl::PointXYZ>);
  //  copyPointCloud(*grid_road_projected,*grid_xyz_projected);

  /************************************************************************************************************************************************************************
   ************************************** find the initial road surface ***************************************************************************************************
   * *********************************************************************************************************************************************************************** */
  //  pcl::search::KdTree<pcl::PointXYZ>::Ptr treeGrid (new pcl::search::KdTree<pcl::PointXYZ> ());
  //  treeGrid->setInputCloud(grid_xyz_projected);
  //  pcl::PointXYZ the_point;
  //  for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_passthrough_road->points.begin (); it != cloud_passthrough_road->points.end (); ++it)
  //  {
  //    pcl::PointXYZ point;
  //    point.x = it->x;
  //    point.y = it->y;
  //    point.z = it->z;
  //    std::vector<int> pointIdx;
  //    std::vector<float> pointDist;
  //    treeGrid->radiusSearch(point,0.01,pointIdx,pointDist,0);
  //    if(pointIdx.size()>0)
  //    {
  //      grid_road_projected->points[pointIdx[0]].roadType = 1;
  //      grid_road_projected->points[pointIdx[0]].probability = 1;
  //      double dist_from_plane = std::abs((model_coeff_pass->values[0]*grid_road_projected->points[pointIdx[0]].x +
  //          model_coeff_pass->values[1]*grid_road_projected->points[pointIdx[0]].y +
  //          model_coeff_pass->values[2]*grid_road_projected->points[pointIdx[0]].z + model_coeff_pass->values[3]))/
  //              (coeffSqrt_pass);
  //      grid_road_projected->points[pointIdx[0]].height = dist_from_plane;
  //      std::cout<<"point found!"<<std::endl;
  //      the_point.x = it->x;
  //      the_point.y = it->y;
  //      the_point.z = it->z;
  //    }
  //  }

  /************************************************************************************************************************************************************************
   ************************************** Translate road type into color rgb value ***********************************************************************************
   * *********************************************************************************************************************************************************************** */
  //      for (int it=0;it<grid_road_projected->points.size();it++)
  //      {
  //        pcl::PointXYZRGB point;
  //        point.x = grid_road_projected->points[it].x;
  //        point.y = grid_road_projected->points[it].y;
  //        point.z = grid_road_projected->points[it].z;
  //        point.r = 0;
  //        point.g = 0;
  //        point.b = 0;
  //        if(grid_road_projected->points[it].roadType == 1)
  //        {
  //          point.g = 255;
  //        } else if(grid_road_projected->points[it].roadType == -1)
  //        {
  //          point.r = 255;
  //        } else
  //        {
  //          point.b = 255;
  //        }
  //        grid_cloud->push_back(point);
  //      }
  //      writer.write<pcl::PointXYZRGB> ("road_height.pcd", *grid_cloud, false);


  //  int counter = 0;
  //  while(++counter<1000)
  //  {
  //
  //  }
  //  //    for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it = grid_cloud_projected->points.begin (); it != grid_cloud_projected->points.end (); ++it)
  //  //      {
  //  //        double dist_from_plane = std::abs((model_coeff_pass->values[0]*it->x +
  //  //            model_coeff_pass->values[1]*it->y +
  //  //            model_coeff_pass->values[2]*it->z + model_coeff_pass->values[3]))/
  //  //                (coeffSqrt_pass);
  //  //        if(dist_from_plane<0.05)
  //  //       {
  //  //        cout <<"cloud_passthrough_road point\t"<<it->x<<","<<it->y<<","<<it->z<< "\t distance:\t"<<dist_from_plane<<std::endl;
  //  //       }
  //  //      }
  //  /*
  //   * Find average distance of points in each grid cell from road plane
  //   */
  //  /*
  //   * First project point cloud on road plane
  //   */
  //  pcl::ProjectInliers<pcl::PointXYZ> proj; // (new pcl::ProjectInliers<pcl::PointXYZ>);
  //  proj.setModelType (pcl::SACMODEL_PLANE);
  //  proj.setInputCloud (cloud_in);
  //  proj.setModelCoefficients (model_coeff_pass);
  //  proj.filter (*cloud_project);
  //  std::cerr << "Cloud after project: " << std::endl;
  //  std::cerr << *cloud_project << std::endl;
  //
  //  //  for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_project->points.begin (); it != cloud_project->points.end (); ++it)
  //  //        {
  //  //          double dist_from_plane = std::abs((model_coeff_pass->values[0]*it->x +
  //  //              model_coeff_pass->values[1]*it->y +
  //  //              model_coeff_pass->values[2]*it->z + model_coeff_pass->values[3]))/
  //  //                  (coeffSqrt_pass);
  //  //          if(dist_from_plane<0.05)
  //  //          {
  //  //          cout <<"cloud_passthrough_road point\t"<<it->x<<","<<it->y<<","<<it->z<< "\t distance:\t"<<dist_from_plane<<std::endl;
  //  //          }
  //  //        }
  //  /*
  //   * Find average height
  //   */
  //  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  //  tree->setInputCloud(cloud_project);
  //
  //  //  for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it = grid_cloud_projected->points.begin (); it != grid_cloud_projected->points.end (); ++it)
  //  for (int it=0;it<grid_road_projected->points.size();it++)
  //  {
  //    double height_sum = 0;
  //    pcl::PointXYZ point;
  //    point.x = grid_road_projected->points[it].x;
  //    point.y = grid_road_projected->points[it].y;
  //    point.z = grid_road_projected->points[it].z;
  //    std::vector<int> pointIdx;
  //    std::vector<float> pointDist;
  //    tree->radiusSearch(point,search_radius,pointIdx,pointDist,0);
  //
  //    pcl::PointCloud<pcl::PointXYZ>::Ptr n_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //    if(pointIdx.size()!=0)
  //    {
  //      for (int i=0;i<pointIdx.size();i++)
  //      {
  //        //        pcl::PointXYZ n_point;
  //        //        n_point.x = it->x;
  //        //        n_point.y = it->y;
  //        //        n_point.z = it->z;
  //        //        n_cloud->push_back(n_point);
  //        double dist_from_road = std::abs((model_coeff_pass->values[0]*cloud_in->points[pointIdx[i]].x +
  //            model_coeff_pass->values[1]*cloud_in->points[pointIdx[i]].y +
  //            model_coeff_pass->values[2]*cloud_in->points[pointIdx[i]].z + model_coeff_pass->values[3]))/
  //                (coeffSqrt_pass);
  //        height_sum += dist_from_road;
  //      }
  //      double avg_height = height_sum/pointIdx.size();
  //      grid_road_projected->points[it].height = avg_height;
  //
  //      if(avg_height<0.1)
  //      {
  //        grid_road_projected->points[it].roadType = 1;
  //        // std::cout <<"height sum:\t"<<height_sum<<"\t average height:\t"<<avg_height<<"\t no of points:\t"<<pointIdx.size()<<"\tflag:\t road"<<std::endl;
  //      } else
  //      {
  //        grid_road_projected->points[it].roadType = -1;
  //        // std::cout <<"height sum:\t"<<height_sum<<"\t average height:\t"<<avg_height<<"\t no of points:\t"<<pointIdx.size()<<"\tflag:\t non-road"<<std::endl;
  //      }
  //    }
  //  }
  //
  //  /*
  //   * Translate road type into color rgb value
  //   */
  //  for (int it=0;it<grid_road_projected->points.size();it++)
  //  {
  //    pcl::PointXYZRGB point;
  //    point.x = grid_road_projected->points[it].x;
  //    point.y = grid_road_projected->points[it].y;
  //    point.z = grid_road_projected->points[it].z;
  //    point.r = 0;
  //    point.g = 0;
  //    point.b = 0;
  //    if(grid_road_projected->points[it].roadType == 1)
  //    {
  //      point.g = 255;
  //    } else if(grid_road_projected->points[it].roadType == -1)
  //    {
  //      point.r = 255;
  //    } else
  //    {
  //      point.b = 255;
  //    }
  //    grid_cloud->push_back(point);
  //  }
  //  writer.write<pcl::PointXYZRGB> ("road_height.pcd", *grid_cloud, false);
  //
  //  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_grid (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  //  tree_grid->setInputCloud(grid_cloud);
  //  int counter=0;
  //  int count = 0;
  //  while(counter<1000000)
  //  {
  //    /* generate secret number between 0 and x_dim*y_dim: */
  //    int grid_size = x_dim*y_dim;
  //    int rand_indx = (rand()%grid_size);
  //    //std::cout <<"random index:\t"<<rand_indx<<std::endl;
  //
  //    //    if(grid_road_projected->points[rand_indx].roadType == 1)
  //    //    {
  //    pcl::PointXYZRGB point;
  //    point.x = grid_cloud->points[rand_indx].x;
  //    point.y = grid_cloud->points[rand_indx].y;
  //    point.z = grid_cloud->points[rand_indx].z;
  //    std::vector<int> pointIdx;
  //    std::vector<float> pointDist;
  //    tree_grid->nearestKSearch(point,9,pointIdx,pointDist);
  //    int road_grid_count = 0;
  //    double n_height_sum = 0;
  //    double n_avg_height = 0;
  //    for (int ii=0;ii<pointIdx.size();ii++)
  //    {
  //      if(grid_road_projected->points[pointIdx[ii]].roadType == 1)
  //      {
  //        //std::cout<<"rand_idx:"<<rand_indx<<"\t road_grid_count:"<<road_grid_count<<std::endl;
  //        road_grid_count++;
  //        n_height_sum += grid_road_projected->points[pointIdx[ii]].height;
  //      }
  //      n_avg_height = n_height_sum/road_grid_count;
  //    }
  //
  //    if(road_grid_count>2)
  //    {
  //      for (int ii=0;ii<pointIdx.size();ii++)
  //      {
  //        if(grid_road_projected->points[pointIdx[ii]].roadType == 0)
  //        {
  //          grid_road_projected->points[pointIdx[ii]].roadType = 1;
  //          grid_road_projected->points[pointIdx[ii]].height = n_avg_height;
  //          count++;
  //          //break;
  //        }else if(grid_road_projected->points[pointIdx[ii]].roadType == -1)
  //        {
  //          if(std::abs(n_avg_height-grid_road_projected->points[ii].height)<0.1)
  //          {
  //            grid_road_projected->points[pointIdx[ii]].roadType = 1;
  //            count++;
  //            //break;
  //          }
  //        }
  //      }
  //    }
  //    //    }
  //    counter++;
  //  }
  //  std::cout <<"no of grid cells modified:\t"<<count<<std::endl;
  //  /************************************************************************************************************************************************************************
  //   * Translate road type into color rgb value
  //   * *********************************************************************************************************************************************************************** */
  //  pcl::PointCloud<pcl::PointXYZRGB>::Ptr grid_cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
  //  for (int it=0;it<grid_road_projected->points.size();it++)
  //  {
  //    pcl::PointXYZRGB point;
  //    point.x = grid_road_projected->points[it].x;
  //    point.y = grid_road_projected->points[it].y;
  //    point.z = grid_road_projected->points[it].z;
  //    point.r = 0;
  //    point.g = 0;
  //    point.b = 0;
  //    if(grid_road_projected->points[it].roadType == 1)
  //    {
  //      point.g = 255;
  //    } else if(grid_road_projected->points[it].roadType == -1)
  //    {
  //      point.r = 255;
  //    } else
  //    {
  //      point.b = 255;
  //    }
  //    grid_cloud_rgb->push_back(point);
  //  }
  //  writer.write<pcl::PointXYZRGB> ("road_bp.pcd", *grid_cloud_rgb, false);
  //
  //  /*
  //   * Belief Propagation
  //   */
  //  //   Eigen::MatrixXf road_probability(x_dim,y_dim);
  //  //   road_probability = Eigen::MatrixXf::Constant(x_dim,y_dim,1);
  //  //   Eigen::MatrixXf road_prob_bak = road_probability;
  //  //
  //  //     pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_grid (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  //  //     tree_grid->setInputCloud(grid_cloud_projected);
  //  //
  //  //   for (int i=0;i<grid_cloud_projected->points.size();i++)
  //  //   {
  //  //     int x_indx = (int)(i/y_dim);
  //  //     int y_indx = (int)(i%y_dim);
  //  //     pcl::PointXYZRGB point;
  //  //     point.x = grid_cloud_projected->points[i].x;
  //  //     point.y = grid_cloud_projected->points[i].y;
  //  //     point.z = grid_cloud_projected->points[i].z;
  //  //
  //  //     std::vector<int> pointIdx;
  //  //     std::vector<float> pointDist;
  //  //     tree_grid->nearestKSearch(point,9,pointIdx,pointDist);
  //  //     for (int ii=0;ii<pointIdx.size();ii++)
  //  //     {
  //  //       if()
  //  //     }
  //  //
  //  //   }
  //
  //
  //
  //  /*
  //   * Project pointcloud on xy-plane
  //   */
  //  /*  Eigen::Vector3f normalXYPlane (0.0,0.0,1.0);
  //  Eigen::Vector3f normalRoadPlane (model_coeff->values[0],model_coeff->values[1],model_coeff->values[2]);
  //  Eigen::Vector3f cNormal = normalRoadPlane.cross(normalXYPlane);
  //  float sNormal = sqrt(cNormal.squaredNorm());
  //  float dNormal = normalRoadPlane.dot(normalXYPlane);
  //  Eigen::Matrix3f cNormalSkewSymm;
  //  cNormalSkewSymm << 0, -cNormal(2), cNormal(1), cNormal(2), 0, -cNormal(0), -cNormal(1), cNormal(0), 0;
  //  Eigen::Matrix3f rot2XYPlane;
  //  rot2XYPlane = Eigen::Matrix3f::Identity(3,3)+cNormalSkewSymm+cNormalSkewSymm*cNormalSkewSymm*((1-dNormal)/pow(sNormal,2));
  //  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  //  transform.translation() << 0, 0.0, model_coeff->values[3];
  //  transform.rotate (rot2XYPlane);
  //  pcl::transformPointCloud (*cloud_in, *grid_cloud_projected, transform);
  //  std::cout <<"points in transformed pointcloud:\t"<<grid_cloud_projected->points.size()<<std::endl;
  //   */
  //  /*
  //   * project on xy-plane
  //   */
  //  /*  pcl::ProjectInliers<pcl::PointXYZ> proj; // (new pcl::ProjectInliers<pcl::PointXYZ>);
  //  //       pcl::ModelCoefficients::Ptr model_coeff;
  //  model_coeff->values[0] = 0;
  //  model_coeff->values[1] = 0;
  //  model_coeff->values[2] = 1;
  //  model_coeff->values[3] = 0;
  //  proj.setModelType (pcl::SACMODEL_PLANE);
  //  proj.setInputCloud (grid_cloud_projected);
  //  proj.setModelCoefficients (model_coeff);
  //  proj.filter (*cloud_project);
  //  //cloud_project = projectCloudOnPlane(cloud_in);
  //  std::cerr << "Cloud after project: " << std::endl;
  //  std::cerr << *cloud_project << std::endl;
  //
  //   *grid_cloud = getGridMap(-20,20,-20,20,400,400);
  //
  //  for (pcl::PointCloud<pcl::PointXYZ>::iterator it = grid_cloud->points.begin (); it != grid_cloud->points.end (); ++it)
  //  {
  //    pcl::PointXYZ point;
  //    point.x = it->x;
  //    point.y = it->y;
  //    point.z = it->z;
  //    std::vector<int> pointIdx;
  //    std::vector<float> pointDist;
  //    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  //    tree->setInputCloud(cloud_project);
  //    tree->radiusSearch(point,search_radius,pointIdx,pointDist,0);
  //
  //    pcl::PointCloud<pcl::PointXYZ>::Ptr n_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //    for (int i=0;i<pointIdx.size();i++)
  //    {
  //      pcl::PointXYZ n_point;
  //      n_point.x = it->x;
  //      n_point.y = it->y;
  //      n_point.z = it->z;
  //      n_cloud->push_back(n_point);
  //      double n_dist = std::abs(it->z);
  //    }
  //  }
  //
  //
  //  // Create the normal estimation class, and pass the input dataset to it
  //  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  //  ne.setInputCloud (grid_cloud_projected);
  //  ne.setSearchMethod (tree);
  //  ne.setRadiusSearch (0.05);
  //  ne.compute (*grid_normals);
  //
  //  std::cerr << "Object cloud after calculating normals: " << std::endl;
  //  std::cerr << *grid_normals << std::endl;
  //
  //  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  //  viewer->setBackgroundColor (0, 0, 0);
  //  // viewer->addPointCloud<pcl::PointXYZ> (grid_cloud_projected, "sample cloud");
  //  // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (grid_cloud_projected, grid_normals, 10, 0.05, "normals");
  //  viewer->addCoordinateSystem (1.0);
  //  viewer->initCameraParameters ();
  //
  //
  //  writer.write<pcl::Normal> ("road_normals.pcd", *grid_normals, false);
  //
  //  while (!viewer->wasStopped ())
  //  {
  //    viewer->spinOnce (100);
  //    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  //  }
  //   */
  return (0);
}
