/*
 * road_segmentation.hpp
 *
 *  Created on: Nov 26, 2015
 *      Author: mudassir
 */

#ifndef ROAD_SEGMENTATION_HPP_
#define ROAD_SEGMENTATION_HPP_

#include <road_segmentation.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <queue>
#include <list>
#include <cmath>
#include <time.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT>
pcl::RoadSegmentation<PointT,NormalT>::RoadSegmentation () :
octree_resolution_ (0.1),
circular_angle_resolution_ (1),
height_flag_ (true),
curvature_flag_ (false),
//     residual_flag_ (false),
theta_threshold_ (10.0f / 180.0f * static_cast<float> (M_PI)),
//     residual_threshold_ (0.05f),
curvature_threshold_ (0.05f),
neighbour_number_ (30),
search_ (),
octree_(),
normals_ (),
cloud_project_(),
point_labels_ (0),
road_normal_(0,0,1),
road_coefficients_(),
point_neighbours_ (0),
rot_xy_2_road_(Eigen::MatrixXf::Identity(3,3))
//     normal_flag_ (true),
//     num_pts_in_segment_ (0),
//     clusters_ (0),
//     number_of_segments_ (0)

{
//  octree_ = typename pcl::octree::OctreePointCloudSearch <PointT>::Ptr (new pcl::octree::OctreePointCloudSearch <PointT>(0.1));
//  octree_->
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT>
pcl::RoadSegmentation<PointT,NormalT>::~RoadSegmentation ()
{
  if (search_ != 0)
    search_.reset ();
  if (normals_ != 0)
    normals_.reset ();

  point_neighbours_.clear ();
  point_labels_.clear ();
  //  num_pts_in_segment_.clear ();
  //    clusters_.clear ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   template <typename PointT, typename NormalT> int
//   pcl::RoadSegmentation<PointT,NormalT>::getMinClusterSize ()
//   {
//     return (min_pts_per_cluster_);
//   }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   template <typename PointT, typename NormalT> void
//   pcl::RoadSegmentation<PointT,NormalT>::setMinClusterSize (int min_cluster_size)
//   {
//     min_pts_per_cluster_ = min_cluster_size;
//   }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   template <typename PointT, typename NormalT> int
//   pcl::RoadSegmentation<PointT,NormalT>::getMaxClusterSize ()
//   {
//     return (max_pts_per_cluster_);
//   }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   template <typename PointT, typename NormalT> void
//   pcl::RoadSegmentation<PointT,NormalT>::setMaxClusterSize (int max_cluster_size)
//   {
//     max_pts_per_cluster_ = max_cluster_size;
//   }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool
pcl::RoadSegmentation<PointT,NormalT>::getCurvatureTestFlag () const
{
  return (curvature_flag_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RoadSegmentation<PointT,NormalT>::setCurvatureTestFlag (bool value)
{
  curvature_flag_ = value;
  //
  //     if (curvature_flag_ == false && residual_flag_ == false)
  //       residual_flag_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   template <typename PointT, typename NormalT> bool
//   pcl::RoadSegmentation<PointT,NormalT>::getResidualTestFlag () const
//   {
//     return (residual_flag_);
//   }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   template <typename PointT, typename NormalT> void
//   pcl::RoadSegmentation<PointT,NormalT>::setResidualTestFlag (bool value)
//   {
//     residual_flag_ = value;
//
//     if (curvature_flag_ == false && residual_flag_ == false)
//       curvature_flag_ = true;
//   }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool
pcl::RoadSegmentation<PointT,NormalT>::getHeightFlag () const
{
  return (height_flag_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RoadSegmentation<PointT,NormalT>::setHeightFlag (bool value)
{
  height_flag_ = value;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> float
pcl::RoadSegmentation<PointT,NormalT>::getSmoothnessThreshold () const
{
  return (theta_threshold_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RoadSegmentation<PointT,NormalT>::setSmoothnessThreshold (float theta)
{
  theta_threshold_ = theta;
}

//   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   template <typename PointT, typename NormalT> float
//   pcl::RoadSegmentation<PointT,NormalT>::getResidualThreshold () const
//   {
//     return (residual_threshold_);
//   }
//
//   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   template <typename PointT, typename NormalT> voidnew pcl::RoadSegmentation<pcl::PointXYZ,pcl::Normal>(
//   pcl::RoadSegmentation<PointT,NormalT>::setResidualThreshold (float residual)
//   {
//     residual_threshold_ = residual;
//   }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> float
pcl::RoadSegmentation<PointT,NormalT>::getCurvatureThreshold () const
{
  return (curvature_threshold_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RoadSegmentation<PointT,NormalT>::setCurvatureThreshold (float curvature)
{
  curvature_threshold_ = curvature;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> unsigned int
pcl::RoadSegmentation<PointT,NormalT>::getNumberOfNeighbours () const
{
  return (neighbour_number_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RoadSegmentation<PointT,NormalT>::setNumberOfNeighbours (unsigned int neighbour_number)
{
  neighbour_number_ = neighbour_number;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> float
pcl::RoadSegmentation<PointT,NormalT>::getOctreeResolution () const
{
  return (octree_resolution_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RoadSegmentation<PointT,NormalT>::setOctreeResolution (float octree_resolution)
{
  octree_resolution_ = octree_resolution;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> typename pcl::RoadSegmentation<PointT,NormalT>::OcTreePtr
pcl::RoadSegmentation<PointT,NormalT>::getSearchMethod () const
{
  return (octree_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RoadSegmentation<PointT,NormalT>::setSearchMethod (const OcTreePtr& tree)
{
  if (octree_ != 0)
    octree_.reset ();

  octree_ = tree;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RoadSegmentation<PointT,NormalT>::setSearchCloud (const PointCloudPtr& projected_cloud)
{
  octree_ = typename pcl::octree::OctreePointCloudSearch <PointT>::Ptr (new pcl::octree::OctreePointCloudSearch <PointT>(octree_resolution_));
//  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_road (octree_resolution)
  octree_->setInputCloud(projected_cloud);
  octree_->addPointsFromInputCloud ();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//      template <typename PointT, typename NormalT> typename pcl::RoadSegmentation<PointT,NormalT>::KdTreePtr
//      pcl::RoadSegmentation<PointT,NormalT>::getSearchMethod () const
//      {
//        return (search_);
//      }
//
//      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//      template <typename PointT, typename NormalT> void
//      pcl::RoadSegmentation<PointT,NormalT>::setSearchMethod (const KdTreePtr& tree)
//      {
//        if (search_ != 0)
//          search_.reset ();
//
//        search_ = tree;
//      }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> typename pcl::RoadSegmentation<PointT,NormalT>::NormalPtr
pcl::RoadSegmentation<PointT,NormalT>::getInputNormals () const
{
  return (normals_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RoadSegmentation<PointT,NormalT>::setInputNormals (const NormalPtr& norm)
{
  if (normals_ != 0)
    normals_.reset ();

  normals_ = norm;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> Eigen::Vector3f
pcl::RoadSegmentation<PointT,NormalT>::getRoadNormal () const
{
  return (road_normal_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RoadSegmentation<PointT,NormalT>::setRoadNormal (Eigen::Vector3f normal)
{
  road_normal_ = normal;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> typename pcl::RoadSegmentation<PointT, NormalT>::ModelCoefficient
pcl::RoadSegmentation<PointT, NormalT>::getRoadPlaneCoefficients () const
{
  return (road_coefficients_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RoadSegmentation<PointT, NormalT>::setRoadPlaneCoefficients (const ModelCoefficient road_coefficients)
{
  road_coefficients_ = road_coefficients;
  setRoadNormal(Eigen::Vector3f(road_coefficients->values[0],road_coefficients->values[1],road_coefficients->values[2]));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RoadSegmentation<PointT, NormalT>::setProjectedCloud  (const PointCloudPtr& project)
{
  cloud_project_ = project;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RoadSegmentation<PointT,NormalT>::detect (std::vector <int>& point_labels)
{

  point_labels_.clear ();
  point_labels.clear();
  //  int num_of_pts = static_cast<int> (indices_->size ());
  point_labels_.resize (input_->points.size (), -1);
  int point_numbers = input_->points.size();
  std::cout << "indices size:" << point_numbers << std::endl;
  bool segmentation_is_possible = initCompute ();
  if ( !segmentation_is_possible )
  {
    deinitCompute ();
    return;
  }

  float cosine_threshold = cosf (theta_threshold_);
  std::cout << "cosine threshold:" << cosine_threshold << std::endl;
  point_labels.resize (indices_->size ());
  Eigen::Vector3f road_normal (0,0,1);
  for (int p_indx = 0; p_indx < point_numbers; p_indx++)
  {
    Eigen::Map<Eigen::Vector3f> p_normal (static_cast<float*> (normals_->points[p_indx].normal));
    float dot_product = fabsf (p_normal.dot (road_normal));
    //  std::cout << "dot product:" << dot_product << std::endl;
    if (dot_product > cosine_threshold)
    {
      point_labels_[p_indx] = 1;
    }
  }

  point_labels = point_labels_;
  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RoadSegmentation<PointT,NormalT>::calculateXY2RoadRotation ()
{
  Eigen::Vector3f normal_xy_plane (0.0,0.0,1.0);
  // Eigen::Vector3f normal_road_plane = road_normal_;
  Eigen::Vector3f c_normal = normal_xy_plane.cross(road_normal_);
  float s_normal = sqrt(c_normal.squaredNorm());
  float d_normal = normal_xy_plane.dot(road_normal_);
  Eigen::Matrix3f c_normal_skew_symm;
  c_normal_skew_symm << 0, -c_normal(2), c_normal(1), c_normal(2), 0, -c_normal(0), -c_normal(1), c_normal(0), 0;
  //  Eigen::Matrix3f rot_from_xy;
  rot_xy_2_road_ = Eigen::Matrix3f::Identity(3,3) + c_normal_skew_symm + c_normal_skew_symm*c_normal_skew_symm * ((1-d_normal)/pow(s_normal,2));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RoadSegmentation<PointT,NormalT>::segment (PointCloud &output)
{
  // clusters_.clear ();
  //  clusters.clear ();
  //  point_neighbours_.clear ();
  //  point_labels_.clear ();
  // num_pts_in_segment_.clear ();
  //  number_of_segments_ = 0;

  bool segmentation_is_possible = initCompute ();
  if ( !segmentation_is_possible )
  {
    deinitCompute ();
    return;
  }

  segmentation_is_possible = prepareForSegmentation ();
  if ( !segmentation_is_possible )
  {
    deinitCompute ();
    return;
  }
  std::cout << "Preparation for segmentation succeeded." << std::endl;

  calculateXY2RoadRotation();
  std::cout << "Calculated rotation from xy-plane to road." << std::endl;

  double coeff_sqrt = std::sqrt(std::pow(road_coefficients_->values[0],2)+std::pow(road_coefficients_->values[1],2)+std::pow(road_coefficients_->values[2],2));
  double const dist_of_plane_origin = road_coefficients_->values[3]/(coeff_sqrt);

  Eigen::Affine3f transform_xy_2_road = Eigen::Affine3f::Identity();
  transform_xy_2_road.translation() << -dist_of_plane_origin*road_coefficients_->values[0], -dist_of_plane_origin*road_coefficients_->values[1], -dist_of_plane_origin*road_coefficients_->values[2];
  transform_xy_2_road.rotate (rot_xy_2_road_);

  Eigen::Vector3f origin_point_in_xy (0.0,0.0,0.0);
  Eigen::Vector3f origin_trans_2_road;
  pcl::transformPoint(origin_point_in_xy, origin_trans_2_road, transform_xy_2_road);

  std::vector<int> road_right;
  for (double theta=0;theta<=360;theta++)
  {
    Eigen::Vector3f xy_vector(cos(theta*M_PI/180.0), sin(theta*M_PI/180.0),0);
    Eigen::Vector3f ray_direction_road;
    ray_direction_road = rot_xy_2_road_*xy_vector;
    //    double dot_product = ray_direction.dot(plane_normal);
    std::vector<int> pointIdx;
    std::vector<float> dist;
    octree_->getIntersectedVoxelIndices(origin_trans_2_road,ray_direction_road,pointIdx,0);

    std::cout << "points in intersected ray:" << pointIdx.size() << std::endl;
        int rCount = 0;
        int gCount = 0;
        double road_start_dist = -1;
        double road_end_dist = -1;
        double obs_start_dist = -1;
        double obs_end_dist = -1;
        for (int i = 0; i < pointIdx.size (); ++i)
        {
          double dist_from_origin = std::sqrt(std::pow(origin_trans_2_road[0]-cloud_project_->points[pointIdx[i]].x,2)+
                                              std::pow(origin_trans_2_road[1]-cloud_project_->points[pointIdx[i]].y,2)+
                                              std::pow(origin_trans_2_road[2]-cloud_project_->points[pointIdx[i]].z,2));
          //     std::cout << "distance from origin:" << dist_from_origin << std::endl;
          if(dist_from_origin < 4)
          {
            continue;
          }
          if(point_labels_[pointIdx[i]]==-1)
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
            if((obs_end_dist-obs_start_dist)>0.5)
            {
              break;
            }
          }else if(point_labels_[pointIdx[i]]==1)
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
            if((road_end_dist-road_start_dist)>5)
            {
              road_right.clear();
              for(int k=0;k<=i;k++)
              {
                point_labels_[pointIdx[k]] = 0;
              }
              //         std::cout<<"right side updated." << std::endl;
              break;
            }
          }
        }
  }

  //  findPointNeighbours ();

  deinitCompute ();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool
pcl::RoadSegmentation<PointT,NormalT>::prepareForSegmentation ()
{
  // if user forgot to pass point cloud or if it is empty
  if ( input_->points.size () == 0 )
    return (false);

  // if user forgot to pass normals or the sizes of point and normal cloud are different
  if ( cloud_project_ == 0 || input_->points.size () != cloud_project_->points.size () )
    return (false);

  // from here we check those parameters that are always valuable
       if (octree_resolution_ <= 0)
       {
         std::cout << "no octree resolution found." << octree_resolution_ <<std::endl;
         return (false);
       }

  setSearchCloud(cloud_project_);
    // if user didn't set search method
//    if (!octree_)
//      octree_.reset (new pcl::octree::OctreePointCloudSearch<PointT>);
  //
  //  if (indices_)
  //  {
  //    if (indices_->empty ())
  //      PCL_ERROR ("[pcl::RoadSegmentation::prepareForSegmentation] Empty given indices!\n");
  //    octree_->setInputCloud (cloud_project_, indices_);
  //    octree_->addPointsFromInputCloud ();
  //  }
  //  else
  //  {
  //    octree_->setInputCloud (cloud_project_);
  //    octree_->addPointsFromInputCloud ();
  //  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RoadSegmentation<PointT,NormalT>::findPointNeighbours ()
{
  int point_number = static_cast<int> (indices_->size ());
  std::vector<int> neighbours;
  std::vector<float> distances;

  point_neighbours_.resize (input_->points.size (), neighbours);

  for (int i_point = 0; i_point < point_number; i_point++)
  {
    int point_index = (*indices_)[i_point];
    neighbours.clear ();
    search_->nearestKSearch (i_point, neighbour_number_, neighbours, distances);
    point_neighbours_[point_index].swap (neighbours);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> pcl::PointCloud<pcl::PointXYZRGB>::Ptr
pcl::RoadSegmentation<PointT,NormalT>::getColoredCloud ()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

  if (!point_labels_.empty ())
  {
    colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();

    colored_cloud->width = input_->width;
    colored_cloud->height = input_->height;
    colored_cloud->is_dense = input_->is_dense;
    for (size_t i_point = 0; i_point < input_->points.size (); i_point++)
    {
      pcl::PointXYZRGB point;
      point.x = *(input_->points[i_point].data);
      point.y = *(input_->points[i_point].data + 1);
      point.z = *(input_->points[i_point].data + 2);
      point.r = 0;
      point.g = 0;
      point.b = 0;
      if(point_labels_[i_point] == 1)
      {
        point.g = 255;
      }else if(point_labels_[i_point] == 0)
      {
        point.b = 255;
      }
      else
      {
        point.r = 255;
      }
      colored_cloud->points.push_back (point);
    }
  }

  return (colored_cloud);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   template <typename PointT, typename NormalT> pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
//   pcl::RoadSegmentation<PointT,NormalT>::getColoredCloudRGBA ()
//   {
//     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colored_cloud;
//
//     if (!clusters_.empty ())
//     {
//       colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGBA>)->makeShared ();
//
//       srand (static_cast<unsigned int> (time (0)));
//       std::vector<unsigned char> colors;
//       for (size_t i_segment = 0; i_segment < clusters_.size (); i_segment++)
//       {
//         colors.push_back (static_cast<unsigned char> (rand () % 256));
//         colors.push_back (static_cast<unsigned char> (rand () % 256));
//         colors.push_back (static_cast<unsigned char> (rand () % 256));
//       }
//
//       colored_cloud->width = input_->width;
//       colored_cloud->height = input_->height;
//       colored_cloud->is_dense = input_->is_dense;
//       for (size_t i_point = 0; i_point < input_->points.size (); i_point++)
//       {
//         pcl::PointXYZRGBA point;
//         point.x = *(input_->points[i_point].data);
//         point.y = *(input_->points[i_point].data + 1);
//         point.z = *(input_->points[i_point].data + 2);
//         point.r = 255;
//         point.g = 0;
//         point.b = 0;
//         point.a = 0;
//         colored_cloud->points.push_back (point);
//       }
//
//       std::vector< pcl::PointIndices >::iterator i_segment;
//       int next_color = 0;
//       for (i_segment = clusters_.begin (); i_segment != clusters_.end (); i_segment++)
//       {
//         std::vector<int>::iterator i_point;
//         for (i_point = i_segment->indices.begin (); i_point != i_segment->indices.end (); i_point++)
//         {
//           int index;
//           index = *i_point;
//           colored_cloud->points[index].r = colors[3 * next_color];
//           colored_cloud->points[index].g = colors[3 * next_color + 1];
//           colored_cloud->points[index].b = colors[3 * next_color + 2];
//         }
//         next_color++;
//       }
//     }
//
//     return (colored_cloud);
//   }

#define PCL_INSTANTIATE_RoadSegmentation(T) template class pcl::RoadSegmentation<T, pcl::Normal>;


#endif /* ROAD_SEGMENTATION_HPP_ */
