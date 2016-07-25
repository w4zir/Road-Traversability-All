/*
 * obstacle_inclusion.hpp
 *
 *  Created on: Feb 23, 2016
 *      Author: mudassir
 */

#ifndef OBSTACLE_INCLUSION_HPP_
#define OBSTACLE_INCLUSION_HPP_

#include <obstacle_inclusion.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::OBSTACLE_INCLUSION<PointT>::OBSTACLE_INCLUSION () :
road_min_x_ (-3),
road_max_x_ (3),
road_min_y_ (-10),
road_max_y_ (10),
negative_obs_distance_ (-0.5),
max_ploygon_side_length_ (0.5),
min_circular_obs_radius_ (0.5),
max_circular_obs_radius_ (2.0),
obstacles_info_ (),
circular_obs_radius_ (),
obstacle_parameters_ (),
cloud_in_ (),
cloud_op_ (),
search_ ()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::OBSTACLE_INCLUSION<PointT>::~OBSTACLE_INCLUSION ()
{
	if (search_ != 0)
		search_.reset ();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::OBSTACLE_INCLUSION<PointT>::setObstacleType (int obs_type)
{
	obs_type_ = obs_type;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::OBSTACLE_INCLUSION<PointT>::setObstacleCount (int obs_count)
{
	obs_count_ = obs_count;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::OBSTACLE_INCLUSION<PointT>::setObstacleParameters (Eigen::MatrixXf parameters)
{
	obstacle_parameters_ = parameters;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::OBSTACLE_INCLUSION<PointT>::randomObstacles ()
{
	bool is_ready = preProcessing();
	if(!is_ready)
	{
		std::cout << "Error, issue in pre-processing for PRM." << std::endl;
		return;
	}

	std::cout << "starting to add obstacles." << std::endl;
	for (int obs_idx=0; obs_idx < obs_count_; obs_idx++)
	{
		int rand_obs_type = rand()%3;
		rand_obs_type = 0;
		//		std::cout << "random no" << rand_obs_type << std::endl;
		if(rand_obs_type == 0)
		{
			float circle_rand_radius = (max_circular_obs_radius_- min_circular_obs_radius_)*rand()/((float)(RAND_MAX)) + min_circular_obs_radius_;
			float circle_center_x = (road_max_x_ - road_min_x_)*(rand()/((float)(RAND_MAX))) + road_min_x_;
			float circle_center_y = (road_max_y_ - road_min_y_)*(rand()/((float)(RAND_MAX))) + road_min_y_;
			circle_rand_radius = std::floor(circle_rand_radius * 100) / 100;
			circle_center_x = std::floor(circle_center_x * 100) / 100;
			circle_center_y = std::floor(circle_center_y * 100) / 100;

			std::cout << "adding circle with center:(" <<circle_center_x <<","<< circle_center_y <<") and radius:"<< circle_rand_radius << std::endl;
			addCircularObstacle(circle_center_x, circle_center_y, circle_rand_radius);
			obstacles_info_.row(obs_idx) = Eigen::Vector3f(circle_center_x, circle_center_y, circle_rand_radius);
		}
	}	
	cloud_op_ = cloud_in_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::OBSTACLE_INCLUSION<PointT>::samePairObstacles ()
{
	bool is_ready = preProcessing();
	if(!is_ready)
	{
		std::cout << "Error, issue in pre-processing for PRM." << std::endl;
		return;
	}

	std::cout << "starting to add obstacles." << std::endl;
	int rand_obs_type = rand()%2;
	rand_obs_type = 0;
	/*
	 * circular obstacle with a radius constraint
	 * center of circle on side of road
	 */
	if(rand_obs_type == 0)
	{
		// obstacle 1 info
		float circle_rand_radius = (max_circular_obs_radius_- min_circular_obs_radius_)*rand()/((float)(RAND_MAX)) + min_circular_obs_radius_;
		float circle_center_x = road_max_x_;
		float circle_center_y = (road_max_x_-road_min_x_)*(rand()/((float)(RAND_MAX))) - (road_max_x_-road_min_x_)/2;
		circle_rand_radius = std::floor(circle_rand_radius * 100) / 100;
		circle_center_x = std::floor(circle_center_x * 100) / 100;
		circle_center_y = std::floor(circle_center_y * 100) / 100;

		// obstacle 2 info
		float circle_rand_radius2 = (max_circular_obs_radius_- min_circular_obs_radius_)*rand()/((float)(RAND_MAX)) + min_circular_obs_radius_;
		float circle_center_x2 = std::floor(-circle_center_x * 100) / 100;			
		float circle_center_y2 = (road_max_x_-road_min_x_)*(rand()/((float)(RAND_MAX))) - (road_max_x_-road_min_x_)/2;
		circle_center_y2 = std::floor(circle_center_y2 * 100) / 100;

		std::cout << "adding circle with center:(" <<circle_center_x <<","<< circle_center_y <<") and radius:"<< circle_rand_radius << std::endl;
		addCircularObstacle(circle_center_x, circle_center_y, circle_rand_radius);
		addCircularObstacle(circle_center_x2, circle_center_y2, circle_rand_radius2);
		obstacles_info_.row(0) = Eigen::Vector3f(circle_center_x, circle_center_y, circle_rand_radius);
		obstacles_info_.row(1) = Eigen::Vector3f(circle_center_x2, circle_center_y2, circle_rand_radius2);
	} else if(rand_obs_type == 1)
	{
		/*
		 * rectangular obstacle with a side constraints
		 */
		// obstacle 1 info
		//		float circle_rand_radius = (max_circular_obs_radius_- min_circular_obs_radius_)*rand()/((float)(RAND_MAX)) + min_circular_obs_radius_;
		//		float circle_center_x = road_max_x;
		//		float circle_center_y = (road_max_y_ - road_min_y_ - 10)*(rand()/((float)(RAND_MAX))) + road_min_y_+5;
		//		circle_rand_radius = std::floor(circle_rand_radius * 100) / 100;
		//		circle_center_x = std::floor(circle_center_x * 100) / 100;
		//		circle_center_y = std::floor(circle_center_y * 100) / 100;
		//
		//		// obstacle 2 info
		//		float circle_rand_radius2 = (max_circular_obs_radius_- min_circular_obs_radius_)*rand()/((float)(RAND_MAX)) + min_circular_obs_radius_;
		//		float circle_center_x2 = std::floor(-circle_center_x * 100) / 100;			
		//		float circle_center_y2 = (road_max_y_ - road_min_y_ - 10)*(rand()/((float)(RAND_MAX))) + road_min_y_+5;
		//		circle_center_y2 = std::floor(circle_center_y2 * 100) / 100;
		//
		//		std::cout << "adding circle with center:(" <<circle_center_x <<","<< circle_center_y <<") and radius:"<< circle_rand_radius << std::endl;
		//		addCircularObstacle(circle_center_x, circle_center_y, circle_rand_radius);
		//		addCircularObstacle(circle_center_x2, circle_center_y2, circle_rand_radius2);
		//		obstacles_info_.row(obs_idx) = Eigen::Vector3f(circle_center_x, circle_center_y, circle_rand_radius);
		//		obstacles_info_.row(obs_idx) = Eigen::Vector3f(circle_center_x2, circle_center_y2, circle_rand_radius2);
	}
	cloud_op_ = cloud_in_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::OBSTACLE_INCLUSION<PointT>::addCircularObstacle (float center_x, float center_y, float radius)
{
	pcl::PointXYZ search_point;
	search_point.x = center_x;
	search_point.y = center_y;
	search_point.z = 0;

	std::vector<int> point_idx;
	std::vector<float> point_dist;

	search_->radiusSearch (search_point, radius, point_idx, point_dist);
	std::cout << "points in circle:" << point_idx.size () << std::endl;
	for (int idx = 0; idx < point_idx.size (); idx++)
	{
		cloud_in_->points[point_idx[idx]].z = negative_obs_distance_; 
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::OBSTACLE_INCLUSION<PointT>::addRectangularObstacle (float min_x, float max_x, float min_y, float max_y)
{	
	for (pcl::PointCloud<PointXYZ>::iterator it = cloud_in_->points.begin (); it != cloud_in_->points.end (); ++it)
	{
		if (it->x >= min_x && it->x <= max_x && it->y >= min_y && it->y <= max_y)
		{
			it->z = negative_obs_distance_;

		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::OBSTACLE_INCLUSION<PointT>::setSearchMethod (const OcTreePtr& tree)
{
	if (search_ != 0)
		search_.reset ();

	search_ = tree;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::OBSTACLE_INCLUSION<PointT>::addPolygon (int sides_count)
{

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::OBSTACLE_INCLUSION<PointT>::setInCloud ()
{
	//	std::cout << "setting in cloud." << input_->points.size() << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
	for (int i=0; i<input_->points.size(); i++)
	{
		cloud_in->push_back(pcl::PointXYZ(input_->points[i].x,input_->points[i].y,input_->points[i].z));
	}
	cloud_in_ = cloud_in;
	//	std::cout << "setting in cloud done." << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::OBSTACLE_INCLUSION<PointT>::preProcessing ()
{
	std::cout << "Pre-processing for obstacle inclusion." << std::endl;

	bool computation_is_possible = initCompute ();
	if ( !computation_is_possible )
	{
		deinitCompute ();
		return false;
	}

	// if user forgot to pass point cloud or if it is empty
	if ( input_->points.size () == 0 )
		return (false);

	// if user didn't set search method
	if (!search_)
		search_.reset (new pcl::octree::OctreePointCloudSearch<PointT>(128.0f));

	setInCloud();
	search_->setInputCloud(cloud_in_);
	search_->addPointsFromInputCloud ();

	obstacles_info_ = Eigen::MatrixXf(obs_count_,3);

	return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::OBSTACLE_INCLUSION<PointT>::getOutputCloud (PointCloudPtr &pointcloud)
{
	pointcloud = cloud_op_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::OBSTACLE_INCLUSION<PointT>::getObstaclesInfo (Eigen::MatrixXf &obstacles_info)
{
	obstacles_info = obstacles_info_;
}


#define PCL_INSTANTIATE_OBSTACLE_INCLUSION(T) template class pcl::OBSTACLE_INCLUSION<T>;

#endif /* OBSTACLE_INCLUSION_HPP_ */
