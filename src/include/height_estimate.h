/*
 * height_estimate.h
 *
 *  Created on: Dec 1, 2015
 *      Author: mudassir
 */

#ifndef HEIGHT_ESTIMATE_H_
#define HEIGHT_ESTIMATE_H_


#include <pcl/pcl_base.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/search/search.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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
  template <typename PointT>
  class PCL_EXPORTS HeightEstimate : public pcl::PCLBase<PointT>
  {
    public:

      typedef pcl::search::Search <PointT> KdTree;
      typedef typename KdTree::Ptr KdTreePtr;
      typedef pcl::PointCloud <PointT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef pcl::ModelCoefficients::Ptr ModelCoefficient;

      using PCLBase <PointT>::input_;
      using PCLBase <PointT>::indices_;
      using PCLBase <PointT>::initCompute;
      using PCLBase <PointT>::deinitCompute;

    public:

      /** \brief Constructor that sets default values for member variables. */
      HeightEstimate ();

      /** \brief This destructor destroys the cloud, normals and search method used for
       * finding KNN. In other words it frees memory.
       */
      virtual
      ~HeightEstimate ();


      /** \brief Returns Road patch normal. */
      ModelCoefficient
      getRoadPlaneCoefficients () const;

      /** \brief Road normal for projection.
       * \param[in] normal of road surface
       */
      void
      setRoadPlaneCoefficients (ModelCoefficient road_coefficients);

      /** \brief Returns the number of nearest neighbours used for KNN. */
      float
      getNeighboursRadius () const;

      /** \brief Allows to set the number of neighbours. For more information check the article.
       * \param[in] neighbour_number number of neighbours to use
       */
      void
      setNeighboursRadius (float neighbour_radius);

      /** \brief Returns the pointer to the search method that is used for KNN. */
      KdTreePtr
      getSearchMethod () const;

      /** \brief Returns the pointer to the projected cloud on road. */
      virtual void
      getProjectedCloud (PointCloudPtr &project);


      /** \brief This method launches the segmentation algorithm and returns the labels of indices that were
       * obtained during the segmentation.
       * \param[out] labels: each label indicate whether a point is part of road or not.
       */
      virtual void
      estimate (PointCloud &output);

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
      findPointNeighbours ();

      /** \brief Project pointcloud on road. */
      virtual void
      projectCloud ();

    protected:

      /** \brief Number of neighbours to find. */
      float neighbour_radius_;

      //      /** \brief Search method that will be used for KNN. */
      KdTreePtr search_;

      /** \brief Vector that store normal of the know road patch. */
      ModelCoefficient road_coefficients_;

      /** \brief Store projected pointcloud. */
      PointCloudPtr cloud_project_;

      /** \brief Contains neighbours of each point. */
      std::vector<std::vector<int> > point_neighbours_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#include <impl/height_estimate.hpp>

#endif /* HEIGHT_ESTIMATE_H_ */
