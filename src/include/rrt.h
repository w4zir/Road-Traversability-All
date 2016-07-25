/*
 * RRT.h
 *
 *  Created on: Feb 16, 2016
 *      Author: mudassir
 */

#ifndef RRT_H_
#define RRT_H_


#include <pcl/pcl_base.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
//#include <pcl/search/search.h>
//#include <pcl/search/octree.h>

//#include <pcl/octree/octree.h>
#include <pcl/octree/octree_search.h>
//#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <list>
#include <math.h>
#include <time.h>
#include <dem.h>

namespace pcl
{
/** \brief
 * Implement Digital Elevation Map from road point cloud data.
 */
template <typename PointT>
class PCL_EXPORTS RRT : public DEM<PointT>
{
public:
	typedef pcl::octree::OctreePointCloudSearch<PointT> OcTree;
	typedef typename OcTree::Ptr OcTreePtr;
	typedef pcl::PointCloud <PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;

	using DEM<PointT>::road_min_x_;
	using DEM<PointT>::road_max_x_;
	using DEM<PointT>::road_min_y_;
	using DEM<PointT>::road_max_y_;
	using DEM<PointT>::search_;
	using DEM<PointT>::dem_clusters_;
	using DEM<PointT>::cloud_dem_;
	using DEM<PointT>::cloud_project_;

	using DEM<PointT>::setRoadPlaneCoefficients;
	using DEM<PointT>::computeDEM;
	using DEM<PointT>::getDEMVisibilityCloud;	
	using PCLBase <PointT>::initCompute;
	using PCLBase <PointT>::deinitCompute;
	//   using DEM<PointT>::cloud_project_;
	//   using DEM<PointT>::cloud_project_;
	//   using DEM<PointT>::input_;

public:

	/** \brief Constructor that sets default values for member variables. */
	RRT ();

	/** \brief This destructor destroys the cloud, normals and search method used for
	 * finding KNN. In other words it frees memory.
	 */
	virtual
	~RRT ();

	/** \brief Set goal biad probability. */
	virtual void
	setGoalBiasProbability (float goal_bias);

	/** \brief Set goal biad probability. */
	virtual float
	getGoalBiasProbability () const;

	/** \brief Set goal biad probability. */
	virtual int
	getConfigCounter () const;

	/** \brief Generate default DEM cells under vehicle body at (0,0,0). */
	std::vector<int>
	getConfigurationValidityStatus ();

	/** \brief Generate default DEM cells under vehicle body at (0,0,0). */
	Eigen::MatrixXf
	getConfigurations ();

	/** \brief Returns dimension of RRT in y direction. */
	void
	computeRRT ();

	/** \brief Get the DEM clusters. */
	virtual void
	getRRTAdjacencyList (std::vector< std::pair<int,int> >& prm_graph);

	/** \brief Cloud for visibility of RRT. */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
	getRRTVisibilityCloud();

protected:

	/** \brief Allows to set search method that will be used for finding KNN.
	 * \param[in] search search method to use
	 */
	void
	setDEMSearchMethod (const OcTreePtr& tree);

	/** \brief Generate random vehicle configuration. */
	virtual Eigen::Vector3f
	generateRandomConfiguration ();

	/** \brief Find nearest config to the random config. */
	virtual int
	findQrandNearestNeighbor (Eigen::Vector3f qrand);

	/** \brief Find distance between two configurations. */
	virtual float
	config_dist (Eigen::Vector3f qrand, Eigen::Vector4f qnear);

	/** \brief Find new configuration closer to q_rand from q_near. */	
	virtual Eigen::Vector4f
	findOptimalConfig (Eigen::Vector3f qrand, Eigen::Vector4f qnear);

	/** \brief Find whether the q_new is in goal area. */
	virtual bool
	checkGoalReached (Eigen::Vector4f q_new);

	/** \brief Check whether q_new is safe or not. */
	virtual int
	checkConfigSafety (Eigen::Vector4f config);

	/** \brief Returns dimension of RRT in y direction. */
	virtual void
	addStartConfigurations();

	/** \brief Returns dimension of RRT in y direction. */
	virtual void
	generateRRTGraph();

	/** \brief Generate all vehicle configurations. */
	virtual bool
	collisionChecker(Eigen::MatrixXf vehicle_state);

	/** \brief Generate all vehicle configurations. */
	virtual int
	collisionAndSafetyChecker(Eigen::MatrixXf vehicle_state);

	/** \brief Check whether dem clusters is under the body of vehicle or not. */
	virtual bool
	checkDemUnderVehicle (int d_idx, Eigen::MatrixXf vehicle_body_state);

	/** \brief Check dem clusters under tyre for collision with vehicle. */
	virtual bool
	tyreClusterCollisionProcessing(int d_idx);

	/** \brief Check dem clusters under body for collision with vehicle. */
	virtual bool
	bodyClusterCollisionProcessing(int d_idx);


	/** \brief Returns dimension of RRT in y direction. */
	virtual Eigen::MatrixXf
	getVehicleState(Eigen::Vector3f v_config);

	/** \brief Returns dimension of RRT in y direction. */
	virtual Eigen::MatrixXf
	getVehicleStateWithClearance(Eigen::Vector3f v_config);

	virtual bool
	preProcessingRRT();

	virtual inline	 double 
	pointDistFromLine (Eigen::Vector3f point, Eigen::MatrixXf point1, Eigen::MatrixXf point2) 
	{
		Eigen::Vector3f d1=point-point1;
		Eigen::Vector3f d2=point-point2;
		Eigen::Vector3f d3=point2-point1;
		Eigen::Vector3f p = d1.cross(d2);
		return p.norm()/d3.norm();
	}	


protected:

	const static float PI_ = 3.14;
	const static float MAX_PHI_ = 40;

	/** \brief Number of configurations attempted before calling it a fail. */
	int configs_limit_;

	/** \brief Number of start configurations added to the graph. */
	int start_configs_;

	/** \brief Counter that stores the number of configurations added to the tree. */
	int config_counter_;

	/** \brief Number of configurations in y dimension. */
	float min_theta_;

	/** \brief Number of configurations in y dimension. */
	float max_theta_;

	/** \brief Vehicle velocity per second. */
	float vehicle_velocity_;

	/** \brief Change in steering angle of the vehicle per second. */
	float vehicle_phi_;

	/** \brief Number of cells in x dimension. */
	float vehicle_width_;

	/** \brief Number of cells in z dimension. */
	float vehicle_length_;

	/** \brief Number of cells in z dimension. */
	float vehicle_heigt_;

	/** \brief Number of cells in z dimension. */
	float wheelbase_;

	/** \brief Number of cells in z dimension. */
	float front_track_;

	/** \brief Number of cells in z dimension. */
	float ground_clearance_;

	/** \brief Number of cells in z dimension. */
	float tyre_width_;

	/** \brief Number of cells in z dimension. */
	float negative_obs_threshold_;

	/** \brief Number of cells in z dimension. */
	float config_search_radius_;

	/** \brief Number of cells in z dimension. */
	int max_config_neighbors_;

	/** \brief Number of cells in z dimension. */
	int sub_config_count_;

	/** \brief Goal tolerance. */
	float goal_tolerance_;

	/** \brief Goal biad probability. */
	float goal_bias_;

	/** \brief Goal tolerance. */
	float vehicle_safety_dist_;

	/** \brief Number of cells in z dimension. */
	bool is_random_config_;

	/** \brief Search method that will be used for KNN. */
	OcTreePtr dem_search_;

	/** \brief Matrix containing all vehicle configurations on road. */
	Eigen::MatrixXf vehicle_configs_;

	/** \brief Collision status of configurations. */
	std::vector<int> config_collision_status_;

	/** \brief Matrix containing vehicle state on road. */
	Eigen::MatrixXf vehicle_state_;

	/** \brief Vector containing egdes of RRT graph. */
	std::vector< std::pair<int,int> > rrt_graph_;


public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
//#ifdef PCL_NO_PRECOMPILE
#include <impl/rrt.hpp>
//#endif
#endif /* RRT_H_ */
