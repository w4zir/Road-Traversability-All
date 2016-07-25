/*
 * height_estimate.hpp
 *
 *  Created on: Dec 1, 2015
 *      Author: mudassir
 */

#ifndef HEIGHT_ESTIMATE_HPP_
#define HEIGHT_ESTIMATE_HPP_

#include <height_estimate.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::HeightEstimate<PointT>::HeightEstimate () :
neighbour_radius_ (0.1),
//search_ (),
cloud_project_(),
point_neighbours_ (0),
road_coefficients_()
{
  search_ =  typename pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>);
  //  search_ = (KdTreePtr (new KdTreePtr));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::HeightEstimate<PointT>::~HeightEstimate ()
{
  if (search_ != 0)
    search_.reset ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> typename pcl::HeightEstimate<PointT>::ModelCoefficient
pcl::HeightEstimate<PointT>::getRoadPlaneCoefficients () const
{
  return (road_coefficients_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::HeightEstimate<PointT>::setRoadPlaneCoefficients (const ModelCoefficient road_coefficients)
{
  road_coefficients_ = road_coefficients;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::HeightEstimate<PointT>::getNeighboursRadius () const
{
  return (neighbour_radius_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::HeightEstimate<PointT>::setNeighboursRadius (float neighbour_radius)
{
  neighbour_radius_ = neighbour_radius;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> typename pcl::HeightEstimate<PointT>::KdTreePtr
pcl::HeightEstimate<PointT>::getSearchMethod () const
{
  return (search_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::HeightEstimate<PointT>::setSearchMethod (const KdTreePtr& tree)
{
  if (search_ != 0)
    search_.reset ();

  search_ = tree;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::HeightEstimate<PointT>::getProjectedCloud (PointCloudPtr &project)
{
  project = cloud_project_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::HeightEstimate<PointT>::projectCloud ()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_project (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ProjectInliers<PointT> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (input_);
  proj.setModelCoefficients (road_coefficients_);
  proj.filter (*cloud_project);
  cloud_project_ = cloud_project;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::HeightEstimate<PointT>::estimate (PointCloud &output)
{
  int point_numbers = input_->points.size();
  std::cout << "indices size:" << point_numbers << std::endl;
  bool segmentation_is_possible = initCompute ();
  if ( !segmentation_is_possible )
  {
    deinitCompute ();
    return;
  }
  projectCloud();
  std::cout << "projection done." << std::endl;
  findPointNeighbours();
  std::cout << "neighbors done." << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr height_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  int nnCount=0;
  int ynCount = 0;
  double coeff_sqrt = std::sqrt(std::pow(road_coefficients_->values[0],2)+std::pow(road_coefficients_->values[1],2)+std::pow(road_coefficients_->values[2],2));
//  std::cout << "coeffSqrt_pass:" << coeff_sqrt << std::endl;
  for(int i_point=0; i_point < point_numbers; i_point++)
  {
    double total_height = 0;
    int neighbor_count = point_neighbours_[i_point].size();
    for (int n_point=0; n_point < neighbor_count ; n_point++)
    {
      int neighbor_indx = point_neighbours_[i_point][n_point];
      double dist_from_road = std::abs((road_coefficients_->values[0]*input_->points[neighbor_indx].x +
          road_coefficients_->values[1]*input_->points[neighbor_indx].y +
          road_coefficients_->values[2]*input_->points[neighbor_indx].z + road_coefficients_->values[3]))/
              (coeff_sqrt);
      total_height += dist_from_road;
    }
    double avg_height = total_height/neighbor_count;
    pcl::PointXYZ point_to_add;
    point_to_add.x = cloud_project_->points[i_point].x + road_coefficients_->values[0]*avg_height;
    point_to_add.y = cloud_project_->points[i_point].y + road_coefficients_->values[1]*avg_height;
    point_to_add.z = cloud_project_->points[i_point].z + road_coefficients_->values[2]*avg_height;
    height_cloud->points.push_back(point_to_add);
  }
  height_cloud->height = 1;
  height_cloud->width = height_cloud->points.size();

  std::cout << "height estimation done." << std::endl;

  output = *height_cloud;
  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::HeightEstimate<PointT>::findPointNeighbours ()
{
  search_->setInputCloud(cloud_project_);
  int point_number = static_cast<int> (input_->size ());
  std::vector<int> neighbours;
  std::vector<float> distances;

  //  std::cout << "neighbour_radius_:" << neighbour_radius_ << std::endl;
  point_neighbours_.resize (input_->points.size (), neighbours);
  for (int i_point = 0; i_point < point_number; i_point++)
  {
    int point_index = (*indices_)[i_point];
//    std::cout << "point:"<<i_point << "\t index:" << point_index << std::endl;
    neighbours.clear ();
    pcl::PointXYZ point;
    point.x = cloud_project_->points[i_point].x;
    point.y = cloud_project_->points[i_point].y;
    point.z = cloud_project_->points[i_point].z;
    search_ ->radiusSearch (i_point, neighbour_radius_, neighbours, distances);
    point_neighbours_[point_index].swap (neighbours);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//template <typename PointT> pcl::PointCloud<pcl::PointXYZRGB>::Ptr
//pcl::HeightEstimate<PointT>::getColoredCloud ()
//{
//  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
//
//  if (!point_labels_.empty ())
//  {
//    colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();
//
//    colored_cloud->width = input_->width;
//    colored_cloud->height = input_->height;
//    colored_cloud->is_dense = input_->is_dense;
//    for (size_t i_point = 0; i_point < input_->points.size (); i_point++)
//    {
//      pcl::PointXYZRGB point;
//      point.x = *(input_->points[i_point].data);
//      point.y = *(input_->points[i_point].data + 1);
//      point.z = *(input_->points[i_point].data + 2);
//      point.r = 0;
//      point.g = 0;
//      point.b = 0;
//      if(point_labels_[i_point] == 1)
//      {
//        point.g = 255;
//      }else
//      {
//        point.r = 255;
//      }
//      colored_cloud->points.push_back (point);
//    }
//  }
//
//  return (colored_cloud);
//}

#define PCL_INSTANTIATE_HeightEstimate(T) template class pcl::HeightEstimate<T>;

#endif /* HEIGHT_ESTIMATE_HPP_ */
