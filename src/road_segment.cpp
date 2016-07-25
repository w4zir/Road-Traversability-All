/*
 * road_segment.cpp

 *
 *  Created on: Oct 5, 2015
 *      Author: mudassir
 */

/*
 * Detect and segment road in a 3D pointcloud data using plane based
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
double static search_radius_ = 0.2;
int const x_dim_ = 200;
int const y_dim_ = 200;
//int const laser_min_radius = 4;
//Eigen::MatrixXf road_height(x_dim,y_dim);

pcl::PointCloud<pcl::PointXYZ> getGridRoad(float x_min,float x_max,float y_min,float y_max,int x_grid_size, int y_grid_size) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_grid (new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "enter function getVehicleCOnfigurations \n";

  float x_res = (x_max-x_min)/(x_grid_size-1); // because start and end are included
  float y_res = (y_max-y_min)/(y_grid_size-1);  // because start and end are included

  for (int i=0;i<=x_grid_size;i++) {
    for (int j=0;j<=y_grid_size;j++) {
      pcl::PointXYZ point;
      point.x = x_min + i*x_res;
      point.y = y_min + j*y_res;
      point.z = 0.0f;
      cloud_grid->points.push_back(point);
    }
  }
  return *cloud_grid;
}

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PCDWriter writer;

  /*******************************************************************************************************************************
   **************************** Get input pointcloud  ****************************************************************************
   *******************************************************************************************************************************/
  pcl::PCDReader reader;
  reader.read ("road_0.pcd", *cloud_in);
  std::cout<<"Input pointcloud has points:" << cloud_in->points.size() << std::endl;

  /*******************************************************************************************************************************
   **************************** Get grid pointcloud and its indices ***************************************************************
   *******************************************************************************************************************************/
  pcl::PointCloud<pcl::PointXYZ>::Ptr grid_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  *grid_cloud = getGridRoad(-20,20,-20,20,x_dim_,y_dim_);

  std::vector<int> indices;
  indices.resize(grid_cloud->points.size());

  /*******************************************************************************************************************************
   **************************** Get neighbors of the grid pointcloud ***********************************************************
   *********************** This will work because z value of grid cloud is zero ************************************************
   *******************************************************************************************************************************/
  std::vector<std::vector<int> > point_neighbours_;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr search_ (new pcl::search::KdTree<pcl::PointXYZ> ());
  search_->setInputCloud(grid_cloud);
  int point_number = static_cast<int> (grid_cloud->points.size());
  std::vector<int> neighbours;
  std::vector<float> distances;

  point_neighbours_.resize (point_number, neighbours);

  for (int i_point = 0; i_point < point_number; i_point++)
  {
    int point_index = i_point;
    neighbours.clear ();
    search_->radiusSearch (i_point, search_radius_, neighbours, distances);
    point_neighbours_[point_index].swap (neighbours);
  }



  return (0);
}

