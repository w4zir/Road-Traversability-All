/*
 * dem.h
 *
 *  Created on: Feb 16, 2016
 *      Author: mudassir
 */

#ifndef DEM_H_
#define DEM_H_


#include <pcl/pcl_base.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
//#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <list>
#include <math.h>
#include <time.h>




namespace pcl
{
/** \brief Store cluster information. */
struct cluster_stats_
{
	float min_dist;
	float max_dist;
	int cluster_size;
};

/** \brief
 * Implement Digital Elevation Map from road point cloud data.
 */
template <typename PointT>
class PCL_EXPORTS DEM : public pcl::PCLBase<PointT>
{
public:

	typedef pcl::search::Search <PointT> KdTree;
	typedef typename KdTree::Ptr KdTreePtr;
	typedef pcl::PointCloud <PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	typedef pcl::ModelCoefficients::Ptr ModelCoefficient;
	typedef cluster_stats_ clusters;
	//	typedef typename clusters::Ptr clusterPtr;

	using PCLBase <PointT>::input_;
	using PCLBase <PointT>::indices_;
	using PCLBase <PointT>::initCompute;
	using PCLBase <PointT>::deinitCompute;

public:

	/** \brief Constructor that sets default values for member variables. */
	DEM ();

	/** \brief This destructor destroys the cloud, normals and search method used for
	 * finding KNN. In other words it frees memory.
	 */
	virtual
	~DEM ();


	/** \brief Returns Road plane coefficients. */
	ModelCoefficient
	getRoadPlaneCoefficients () const;

	/** \brief Road plane coefficients.
	 * \param[in] normal of road surface
	 */
	void
	setRoadPlaneCoefficients (ModelCoefficient road_coefficients);

	/** \brief Returns dimension of DEM in x direction. */
	int
	getDimensionX () const;

	/** \brief Allows to set the dimension of DEM in x direction. For more information check the article.
	 * \param[in] dimension_x dimension of x.
	 */
	void
	setDimensionX (int dimension_x);

	/** \brief Returns dimension of DEM in y direction. */
	int
	getDimensionY () const;

	/** \brief Allows to set the dimension of DEM iny direction. For more information check the article.
	 * \param[in] dimension_y dimension of y.
	 */
	void
	setDimensionY (int dimension_y);

	/** \brief Returns max distance allowed from road. */
	float
	getMaxDistanceFromRoad () const;

	/** \brief Allows to set the max distance allowed from road.*/
	void
	setMaxDistanceFromRoad (float max_road_dist);


	/** \brief Returns max distance allowed between two clusters. */
	float
	getClusterSegregationDistance () const;

	/** \brief Allows to set the max distance allowed between two clusters.*/
	void
	setClusterSegregationDistance (float cluster_seg_dist);

	/** \brief Returns whether project cloud is provided. */
	bool
	getIsProjectedCloud () const;

	/** \brief Allows to set that project cloud is provided.*/
	void
	setIsProjectCloud (bool is_projected);

	/** \brief Returns the number of nearest neighbours used for KNN. */
	float
	getNeighboursRadius () const;

	/** \brief Allows to set the number of neighbours. For more information check the article.
	 * \param[in] neighbour_number number of neighbours to use
	 */
	void
	setNeighboursRadius (float neighbour_radius);

	/** \brief Returns projected plane coefficients. */
	ModelCoefficient
	getProjectedPlaneCoefficients () const;

	/** \brief Set projected plane coefficients. */
	void
	SetProjectedPlaneCoefficients (ModelCoefficient proj_plane_coefficients);

	/** \brief Returns the pointer to the search method that is used for KNN. */
	KdTreePtr
	getSearchMethod () const;

	/** \brief Returns the pointer to the projected cloud on road. */
	virtual void
	getProjectedCloud (PointCloudPtr &project);

	/** \brief Set the projected cloud on plane. */
	virtual void
	setProjectedCloud (PointCloudPtr project);

	/** \brief Get the DEM clusters. */
	virtual void
	getDEMClusters (std::vector < std::vector < clusters> >& dem_clusters);

	/** \brief Get the DEM cloud. */
	virtual void
	getDEMCloud (PointCloud &pointcloud);

	/** \brief Get the DEM cloud. */
	virtual void
	getMinMax ();

	/** \brief Get the DEM cloud. */
	virtual void
	transformCloud2XYPlane ();
	/** \brief This method launches the segmentation algorithm and returns the labels of indices that were
	 * obtained during the segmentation.
	 * \param[out] labels: each label indicate whether a point is part of road or not.
	 */
	virtual void
	computeDEM ();
	//estimate (PointCloud &output);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
	getDEMVisibilityCloud();
	/** \brief If the cloud was successfully segmented, then function
	 * returns colored cloud. Otherwise it returns an empty pointer.
	 * Points that belong to the same segment have the same color.
	 * But this function doesn't guarantee that different segments will have different
	 * color(it all depends on RNG). Points that were not listed in the indices array will have red color.
	 */
	//      pcl::PointCloud<pcl::PointXYZRGB>::Ptr
	//      getColoredCloud ();

protected:

	/** \brief Allows to set search method that will be used for finding KNN.
	 * \param[in] search search method to use
	 */
	void
	setSearchMethod (const KdTreePtr& tree);

	virtual void
	computeCellClusters (std::vector< std::pair<float, int> > p_dist, int p_indx);

	virtual bool
	preProcessing();

	virtual void
	generateEmptyDEM();

	virtual void
	findPointNeighbours ();

	/** \brief Project pointcloud on road. */
	virtual void
	projectPointCloud ();

protected:

	/** \brief Number of cells in x dimension. */
	float dimension_x_;

	/** \brief Number of cells in z dimension. */
	float dimension_y_;

	/** \brief Width of the road patch. */
	float road_min_x_;

	/** \brief Width of the road patch. */
	float road_max_x_;

	/** \brief Width of the road patch. */
	float road_min_y_;

	/** \brief Width of the road patch. */
	float road_max_y_;

	/** \brief Width of the road patch. */
	float road_width_;

	/** \brief Length of the road patch. */
	float road_length_;

	/** \brief Distance bound above ground plane. */
	float distance_max_;

	/** \brief Distance for cluster segregation. */
	float distance_cluster_;

	/** \brief Whether the input cloud is projected. */
	bool is_projected_;

	/** \brief Coefficients of the plane to project cloud on. */
	ModelCoefficient projected_plane_coeff_;

	/** \brief Radius for DEM cell. */
	float neighbour_radius_;

	//      /** \brief Search method that will be used for KNN. */
	KdTreePtr search_;

	/** \brief Vector that store normal of the know road patch. */
	ModelCoefficient road_coefficients_;

	/** \brief Store projected pointcloud. */
	PointCloudPtr cloud_project_;

	/** \brief Store DEM as pointcloud. */
	PointCloudPtr cloud_dem_;

	/** \brief Contains neighbours of each point. */
	std::vector<std::vector<int> > point_neighbours_;

	/** \brief Contains min and max distance of each cluster in a DEM cell. */
	//	std::vector < std::vector < std::tuple<float, float, int> > > dem_clusters_;
	std::vector < std::vector < clusters > > dem_clusters_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
/** \brief This function is used as a comparator for sorting. */
inline bool
comparePair (std::pair<float, int> i, std::pair<float, int> j)
{
	return (i.first < j.first);
}

}
//#ifdef PCL_NO_PRECOMPILE
#include <impl/dem.hpp>
//#endif
#endif /* DEM_H_ */
