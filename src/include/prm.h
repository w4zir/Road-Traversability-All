/*
 * PRM.h
 *
 *  Created on: Feb 16, 2016
 *      Author: mudassir
 */

#ifndef PRM_H_
#define PRM_H_


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
class PCL_EXPORTS PRM : public DEM<PointT>
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
	PRM ();

	/** \brief This destructor destroys the cloud, normals and search method used for
	 * finding KNN. In other words it frees memory.
	 */
	virtual
	~PRM ();


	/** \brief Returns dimension of PRM in x direction. */
	int
	getConfigurationX () const;

	/** \brief Allows to set the dimension of PRM in x direction. For more information check the article.
	 * \param[in] dimension_x dimension of x.
	 */
	void
	setConfigurationX (int config_x);

	/** \brief Returns dimension of PRM in y direction. */
	int
	getConfigurationY () const;

	/** \brief Allows to set the dimension of PRM iny direction. For more information check the article.
	 * \param[in] dimension_y dimension of y.
	 */
	void
	setConfigurationY (int config_y);

	/** \brief Returns dimension of PRM in y direction. */
	bool
	getRandomConfigFlag () const;

	/** \brief Allows to set the dimension of PRM iny direction. For more information check the article.
	 * \param[in] dimension_y dimension of y.
	 */
	void
	setRandomConfigFlag (bool random_config_flag);

	/** \brief Returns dimension of PRM in x direction. */
	int
	getConfigurationTheta () const;

	/** \brief Allows to set the dimension of PRM in x direction. For more information check the article.
	 * \param[in] dimension_x dimension of x.
	 */
	void
	setConfigurationTheta (int config_theta);

	/** \brief Generate default DEM cells under vehicle body at (0,0,0). */
	void
	defaultDEMCellsCovered ();

	/** \brief Transform DEM cells under vehicle body according to vehicle position and orientation. */
	void
	transformDEMCellsCovered ();

	/** \brief Generate default DEM cells under vehicle body at (0,0,0). */
	std::vector<int>
	getConfigurationValidityStatus ();

	/** \brief Generate default DEM cells under vehicle body at (0,0,0). */
	Eigen::MatrixXf
	getConfigurations ();

	//	/** \brief Returns dimension of PRM in y direction. */
	//	void
	//	computePRM ();

	/** \brief Returns dimension of PRM in y direction. */
	void
	computePRM ();

	/** \brief Get the DEM clusters. */
	virtual void
	getPRMAdjacencyList (std::vector< std::pair<int,int> >& prm_graph);

	/** \brief Cloud for visibility of PRM. */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
	getPRMVisibilityCloud();

protected:

	/** \brief Allows to set search method that will be used for finding KNN.
	 * \param[in] search search method to use
	 */
	void
	setDEMSearchMethod (const OcTreePtr& tree);

	/** \brief Generate PRM cloud from matrix. Used for search for neighbors configurations. */
	virtual void
	getConfigCloud ();

	/** \brief Generate all vehicle configurations. */
	virtual void
	generateConfigurations ();

	/** \brief Returns dimension of PRM in y direction. */
	virtual void
	generateRandomConfigurations();

	/** \brief Returns dimension of PRM in y direction. */
	virtual Eigen::MatrixXf
	getVehicleState(Eigen::Vector3f v_config);

	/** \brief Returns dimension of PRM in y direction. */
	virtual void
	validateConfigurations();

	/** \brief Generate all vehicle configurations. */
	virtual bool
	collisionChecker(Eigen::MatrixXf vehicle_state);

	virtual bool
	preProcessingPRM();

	/** \brief Check dem clusters under tyre for collision with vehicle. */
	virtual bool
	tyreClusterCollisionProcessing(int d_idx);

	/** \brief Check dem clusters under body for collision with vehicle. */
	virtual bool
	bodyClusterCollisionProcessing(int d_idx);

	/** \brief Check whether dem clusters is under the body of vehicle or not. */
	virtual bool
	checkDemUnderVehicle (int d_idx, Eigen::MatrixXf vehicle_body_state);

	/** \brief Check whether dem clusters is under the body of vehicle or not. */
	virtual bool
	findConfigsConnectivity (int c_idx, int n_idx);
	
	/** \brief Returns dimension of PRM in y direction. */
	virtual void
	generatePRMGraph();

	virtual std::vector<int> 	 
	getTempNeighbors (int c_idx);

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

	/** \brief Number of cells in x dimension. */
	int config_x_count_;

	/** \brief Number of configurations in y dimension. */
	int config_y_count_;

	/** \brief Number of configurations in y dimension. */
	int config_theta_count_;

	/** \brief Number of configurations in y dimension. */
	float min_theta_;

	/** \brief Number of configurations in y dimension. */
	float max_theta_;

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

	/** \brief Number of cells in z dimension. */
	bool is_random_config_;

	//      /** \brief Search method that will be used for KNN. */
	OcTreePtr dem_search_;

	/** \brief Store projected pointcloud. */
	PointCloudPtr cloud_prm_;

	/** \brief Matrix containing all vehicle configurations on road. */
	Eigen::MatrixXf vehicle_configs_;

	/** \brief Default DEM cells under vehicle at (0,0,90). */
	Eigen::MatrixXf default_dem_cells_;

	/** \brief Collision status of configurations. */
	std::vector<int> config_collision_status_;

	/** \brief Matrix containing all vehicle configurations on road. */
	Eigen::MatrixXf vehicle_state_;

	/** \brief Vector containing egdes of PRM graph. */
	std::vector< std::pair<int,int> > prm_graph_;


public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
//#ifdef PCL_NO_PRECOMPILE
#include <impl/prm.hpp>
//#endif
#endif /* PRM_H_ */
