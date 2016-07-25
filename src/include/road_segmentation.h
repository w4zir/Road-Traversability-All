/*
 * road_segmentation.h
 *
 *  Created on: Nov 26, 2015
 *      Author: mudassir
 */

#ifndef ROAD_SEGMENTATION_H_
#define ROAD_SEGMENTATION_H_


#include <pcl/pcl_base.h>
#include <pcl/search/search.h>
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>

#include <list>
#include <math.h>
#include <time.h>

namespace pcl
{
  /** \brief
   * Implements the well known Region Growing algorithm used for segmentation.
   * Description can be found in the article
   * "Segmentation of point clouds using smoothness constraint"
   * by T. Rabbania, F. A. van den Heuvelb, G. Vosselmanc.
   * In addition to residual test, the possibility to test curvature is added.
   */
  template <typename PointT, typename NormalT>
  class PCL_EXPORTS RoadSegmentation : public pcl::PCLBase<PointT>
  {
    public:

      typedef pcl::search::Search <PointT> KdTree;
      typedef typename KdTree::Ptr KdTreePtr;
      typedef pcl::octree::OctreePointCloudSearch <PointT> OcTree;
      typedef typename OcTree::Ptr OcTreePtr;
      typedef pcl::PointCloud <NormalT> Normal;
      typedef typename Normal::Ptr NormalPtr;
      typedef pcl::PointCloud <PointT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef pcl::ModelCoefficients::Ptr ModelCoefficient;

      using PCLBase <PointT>::input_;
      using PCLBase <PointT>::indices_;
      using PCLBase <PointT>::initCompute;
      using PCLBase <PointT>::deinitCompute;

    public:

      /** \brief Constructor that sets default values for member variables. */
      RoadSegmentation ();

      /** \brief This destructor destroys the cloud, normals and search method used for
       * finding KNN. In other words it frees memory.
       */
      virtual
      ~RoadSegmentation ();

      /** \brief Get the minimum number of points that a cluster needs to contain in order to be considered valid. */
      float
      getOctreeResolution () const;

      /** \brief Set the minimum number of points that a cluster needs to contain in order to be considered valid. */
      void
      setOctreeResolution (float octree_resolution);

      //      /** \brief Get the maximum number of points that a cluster needs to contain in order to be considered valid. */
      //      int
      //      getMaxClusterSize ();
      //
      //      /** \brief Set the maximum number of points that a cluster needs to contain in order to be considered valid. */
      //      void
      //      setMaxClusterSize (int max_cluster_size);

      //      /** \brief Returns the flag value. This flag signalizes which mode of algorithm will be used.
      //       * If it is set to true than it will work as said in the article. This means that
      //       * it will be testing the angle between normal of the current point and it's neighbours normal.
      //       * Otherwise, it will be testing the angle between normal of the current point
      //       * and normal of the initial point that was chosen for growing new segment.
      //       */
      //      bool
      //      getSmoothModeFlag () const;
      //
      //      /** \brief This function allows to turn on/off the smoothness constraint.
      //       * \param[in] value new mode value, if set to true then the smooth version will be used.
      //       */
      //      void
      //      setSmoothModeFlag (bool value);

      /** \brief Returns the flag that signalize if the curvature test is turned on/off. */
      bool
      getHeightFlag () const;

      /** \brief Allows to turn on/off the curvature test. Note that at least one test
       * (residual or curvature) must be turned on. If you are turning curvature test off
       * then residual test will be turned on automatically.
       * \param[in] value new value for curvature test. If set to true then the test will be turned on
       */
      virtual void
      setHeightFlag (bool value);

      /** \brief Returns the flag that signalize if the curvature test is turned on/off. */
      bool
      getCurvatureTestFlag () const;

      /** \brief Allows to turn on/off the curvature test. Note that at least one test
       * (residual or curvature) must be turned on. If you are turning curvature test off
       * then residual test will be turned on automatically.
       * \param[in] value new value for curvature test. If set to true then the test will be turned on
       */
      virtual void
      setCurvatureTestFlag (bool value);

      //      /** \brief Returns the flag that signalize if the residual test is turned on/off. */
      //      bool
      //      getResidualTestFlag () const;
      //
      //      /** \brief
      //       * Allows to turn on/off the residual test. Note that at least one test
      //       * (residual or curvature) must be turned on. If you are turning residual test off
      //       * then curvature test will be turned on automatically.
      //       * \param[in] value new value for residual test. If set to true then the test will be turned on
      //       */
      //      virtual void
      //      setResidualTestFlag (bool value);
      //
      /** \brief Returns smoothness threshold. */
      float
      getSmoothnessThreshold () const;

      /** \brief Allows to set smoothness threshold used for testing the points.
       * \param[in] theta new threshold value for the angle between normals
       */
      void
      setSmoothnessThreshold (float theta);

      //      /** \brief Returns residual threshold. */
      //      float
      //      getResidualThreshold () const;
      //
      //      /** \brief Allows to set residual threshold used for testing the points.
      //       * \param[in] residual new threshold value for residual testing
      //       */
      //      void
      //      setResidualThreshold (float residual);

      /** \brief Returns curvature threshold. */
      float
      getCurvatureThreshold () const;

      /** \brief Allows to set curvature threshold used for testing the points.
       * \param[in] curvature new threshold value for curvature testing
       */
      void
      setCurvatureThreshold (float curvature);

      /** \brief Returns the number of nearest neighbours used for KNN. */
      unsigned int
      getNumberOfNeighbours () const;

      /** \brief Allows to set the number of neighbours. For more information check the article.
       * \param[in] neighbour_number number of neighbours to use
       */
      void
      setNumberOfNeighbours (unsigned int neighbour_number);

      /** \brief Returns the pointer to the search method that is used for KNN. */
      OcTreePtr
      getSearchMethod () const;

      /** \brief Allows to set search method that will be used for finding KNN.
       * \param[in] search search method to use
       */
      void
      setSearchMethod (const OcTreePtr& tree);

      /** \brief Returns normals. */
      NormalPtr
      getInputNormals () const;

      /** \brief This method sets the normals. They are needed for the algorithm, so if
       * no normals will be set, the algorithm would not be able to segment the points.
       * \param[in] norm normals that will be used in the algorithm
       */
      void
      setInputNormals (const NormalPtr& norm);

      /** \brief Returns Road plane coefficients. */
      ModelCoefficient
      getRoadPlaneCoefficients () const;

      /** \brief Road plane coefficients.
       * \param[in] plane coefficients
       */
      void
      setRoadPlaneCoefficients (ModelCoefficient road_coefficients);

      /** \brief Returns Road patch normal. */
      Eigen::Vector3f
      getRoadNormal () const;

      /** \brief Set projected pointcloud on road plane.
       * \param[in] project pointcloud
       */
      void
      setProjectedCloud (const PointCloudPtr& project);

      /** \brief This method launches the segmentation algorithm and returns the labels of indices that were
       * obtained during the segmentation.
       * \param[out] labels: each label indicate whether a point is part of road or not.
       */
      virtual void
      detect (std::vector <int>& point_labels);

      //      /** \brief This method launches the segmentation algorithm and returns the clusters that were
      //       * obtained during the segmentation.
      //       * \param[out] clusters clusters that were obtained. Each cluster is an array of point indices.
      //       */
      virtual void
      segment (PointCloud &output);

      //      /** \brief For a given point this function builds a segment to which it belongs and returns this segment.
      //       * \param[in] index index of the initial point which will be the seed for growing a segment.
      //       * \param[out] cluster cluster to which the point belongs.
      //       */
      //      virtual void
      //      getSegmentFromPoint (int index, pcl::PointIndices& cluster);

      /** \brief If the cloud was successfully segmented, then function
       * returns colored cloud. Otherwise it returns an empty pointer.
       * Points that belong to the same segment have the same color.
       * But this function doesn't guarantee that different segments will have different
       * color(it all depends on RNG). Points that were not listed in the indices array will have red color.
       */
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      getColoredCloud ();

      //      /** \brief If the cloud was successfully segmented, then function
      //       * returns colored cloud. Otherwise it returns an empty pointer.
      //       * Points that belong to the same segment have the same color.
      //       * But this function doesn't guarantee that different segments will have different
      //       * color(it all depends on RNG). Points that were not listed in the indices array will have red color.
      //       */
      //      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
      //      getColoredCloudRGBA ();

    protected:

      /** \brief Allows to set curvature threshold used for testing the points.
       * \param[in] curvature new threshold value for curvature testing
       */
      void
      setRoadNormal (Eigen::Vector3f road_normal);

      void
      setSearchCloud (const PointCloudPtr& project);

      //      /** \brief This method simply checks if it is possible to execute the segmentation algorithm with
      //       * the current settings. If it is possible then it returns true.
      //       */
      //      virtual bool
      //      prepareForSegmentation ();

      /** \brief This method finds KNN for each point and saves them to the array
       * because the algorithm needs to find KNN a few times.
       */
      virtual void
      findPointNeighbours ();

      virtual void
      calculateXY2RoadRotation();

      virtual bool
      prepareForSegmentation();

      //      /** \brief This function implements the algorithm described in the article
      //       * "Segmentation of point clouds using smoothness constraint"
      //       * by T. Rabbania, F. A. van den Heuvelb, G. Vosselmanc.
      //       */
      //      void
      //      applySmoothRegionGrowingAlgorithm ();
      //
      //      /** \brief This method grows a segment for the given seed point. And returns the number of its points.
      //       * \param[in] initial_seed index of the point that will serve as the seed point
      //       * \param[in] segment_number indicates which number this segment will have
      //       */
      //      int
      //      growRegion (int initial_seed, int segment_number);
      //
      //      /** \brief This function is checking if the point with index 'nghbr' belongs to the segment.
      //       * If so, then it returns true. It also checks if this point can serve as the seed.
      //       * \param[in] initial_seed index of the initial point that was passed to the growRegion() function
      //       * \param[in] point index of the current seed point
      //       * \param[in] nghbr index of the point that is neighbour of the current seed
      //       * \param[out] is_a_seed this value is set to true if the point with index 'nghbr' can serve as the seed
      //       */
      //      virtual bool
      //      validatePoint (int initial_seed, int point, int nghbr, bool& is_a_seed) const;
      //
      //      /** \brief This function simply assembles the regions from list of point labels.
      //       * \param[out] clusters clusters that were obtained during the segmentation process.
      //       * Each cluster is an array of point indices.
      //       */
      //      void
      //      assembleRegions ();

    protected:

      /** \brief Stores resolution used by octree. */
      float octree_resolution_;

      /** \brief Stores angular resolution used for road search. */
      int circular_angle_resolution_;

      //      /** \brief Flag that signalizes if the smoothness constraint will be used. */
      //      bool smooth_mode_flag_;
      //
      //      /** \brief If set to true then curvature test will be done during segmentation. */
      bool curvature_flag_;

      /** \brief If set to true then height of each point from road is calculated. */
      bool height_flag_;

      /** \brief Thershold used for testing the smoothness between points. */
      float theta_threshold_;

      //      /** \brief Thershold used in residual test. */
      //      float residual_threshold_;
      //
      //      /** \brief Thershold used in curvature test. */
      float curvature_threshold_;

      /** \brief Number of neighbours to find. */
      unsigned int neighbour_number_;

      //      /** \brief Search method that will be used for KNN. */
      KdTreePtr search_;

      /** \brief Search method that will be used for circular search. */
      OcTreePtr octree_;

      /** \brief Contains normals of the points that will be segmented. */
      NormalPtr normals_;

      /** \brief Contains neighbours of each point. */
      std::vector<std::vector<int> > point_neighbours_;

      /** \brief Point labels that tells to which segment each point belongs. */
      std::vector<int> point_labels_;

      /** \brief Vector that store normal of the know road patch. */
      Eigen::Vector3f road_normal_;

      /** \brief Rotation matrix that transform from xy-plane to road plane. */
      Eigen::Matrix3f rot_xy_2_road_;

      /** \brief Plane coefficients of the road. */
      ModelCoefficient road_coefficients_;

      /** \brief Store projected pointcloud. */
      PointCloudPtr cloud_project_;


      /** \brief If set to true then normal/smoothness test will be done during segmentation.
       * It is always set to true for the usual region growing algorithm. It is used for turning on/off the test
       * for smoothness in the child class RegionGrowingRGB.*/
      //      bool normal_flag_;

      /** \brief Tells how much points each segment contains. Used for reserving memory. */
      //      std::vector<int> num_pts_in_segment_;

      /** \brief After the segmentation it will contain the segments. */
      //      std::vector <pcl::PointIndices> clusters_;

      /** \brief Stores the number of segments. */
      //      int number_of_segments_;

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
#include <impl/road_segmentation.hpp>
//#endif


#endif /* ROAD_SEGMENTATION_H_ */
