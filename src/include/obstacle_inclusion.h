/*
 * obstacle_inclusion.h
 *
 *  Created on: Feb 16, 2016
 *      Author: mudassir
 */

#ifndef OBSTACLE_INCLUSION_H_
#define OBSTACLE_INCLUSION_H_

#include <pcl/pcl_base.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>

namespace pcl
{

/** \brief
 * Implement Digital Elevation Map from road point cloud data.
 */
template <typename PointT>
class PCL_EXPORTS OBSTACLE_INCLUSION : public pcl::PCLBase<PointT>
{
public:

	typedef pcl::octree::OctreePointCloudSearch<PointT> OcTree;
	typedef typename OcTree::Ptr OcTreePtr;
	typedef pcl::PointCloud <PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	typedef typename PointCloud::ConstPtr PointCloudConstPtr;
	//	typedef typename std::vector< float,float,float > obsInfo;

	using PCLBase <PointT>::input_;
	using PCLBase <PointT>::indices_;
	using PCLBase <PointT>::initCompute;
	using PCLBase <PointT>::deinitCompute;
	//    using PCLBase <PointT>::getInputCloud;
public:

	/** \brief Constructor that sets default values for member variables. */
	OBSTACLE_INCLUSION ();

	/** \brief This destructor destroys the cloud, normals and search method used for
	 * finding KNN. In other words it frees memory.
	 */
	virtual
	~OBSTACLE_INCLUSION ();

	void
	setObstacleType (int obs_type);

	void
	setObstacleCount (int obs_count);

	void
	setObstacleType (int obs_type, int obs_cout);

	void
	setObstacleParameters (Eigen::MatrixXf obstacle_parameters);

	void
	setObstacleParameters (float circular_obs_radius);

	void
	randomObstacles ();
	
	void
	samePairObstacles ();
	
	void
	addCircularObstacle (float center_x, float center_y, float radius);
	
	void
		addRectangularObstacle (float min_x, float max_x, float min_y, float max_y);

	virtual void
	getOutputCloud (PointCloudPtr &pointcloud);

	virtual void
	getObstaclesInfo (Eigen::MatrixXf &obstacles_info);

protected:
	/** \brief Allows to set search method that will be used for finding KNN.
	 * \param[in] search search method to use
	 */
	void
	setSearchMethod (const OcTreePtr& tree);

	virtual bool
	preProcessing();

	void
	addPolygon (int sides);

	virtual void
	setInCloud ();

protected:

	/** \brief Number of cells in x dimension. */
	int cloud_points_cout_;

	/** \brief Width of the road patch. */
	float road_min_x_;

	/** \brief Width of the road patch. */
	float road_max_x_;

	/** \brief Width of the road patch. */
	float road_min_y_;

	/** \brief Width of the road patch. */
	float road_max_y_;

	/** \brief Number of cells in x dimension. */
	int obs_type_;

	/** \brief Number of cells in x dimension. */
	int obs_count_;

	/** \brief Radius for DEM cell. */
	Eigen::MatrixXf polygon_obs_corners_;

	/** \brief Radius for DEM cell. */
	float circular_obs_radius_;

	/** \brief Minimum cricular obstacle radius. */
	float min_circular_obs_radius_;

	/** \brief Maximum cricular obstacle radius. */
	float max_circular_obs_radius_;

	/** \brief Radius for DEM cell. */
	float max_ploygon_side_length_;

	/** \brief Radius for DEM cell. */
	float negative_obs_distance_;

	//      /** \brief Search method that will be used for KNN. */
	OcTreePtr search_;

	/** \brief Radius for DEM cell. */
	Eigen::MatrixXf obstacle_parameters_;

	/** \brief Output pointcloud with obstacles. */
	PointCloudPtr cloud_in_;

	/** \brief Output pointcloud with obstacles. */
	PointCloudPtr cloud_op_;

	/** \brief Store obstacle information. */
	Eigen::MatrixXf obstacles_info_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
//#ifdef PCL_NO_PRECOMPILE
#include <impl/obstacle_inclusion.hpp>
//#endif
#endif /* OBSTACLE_INCLUSION_H_ */
