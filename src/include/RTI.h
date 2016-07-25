/*
 * RTI.h
 *
 *  Created on: Dec 1, 2015
 *      Author: mudassir
 */

#ifndef RTI_H_
#define RTI_H_

#include <pcl/pcl_base.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/search/search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <list>
#include <math.h>
#include <time.h>

namespace pcl
{
  template <typename PointT>
  class PCL_EXPORTS RTI : public pcl::PCLBase<PointT>
  {
    public:

      typedef pcl::search::Search <PointT> KdTree;
      typedef typename KdTree::Ptr KdTreePtr;
      typedef pcl::PointCloud <PointT> PointCloud;

      using PCLBase <PointT>::input_;
      using PCLBase <PointT>::indices_;
      using PCLBase <PointT>::initCompute;
      using PCLBase <PointT>::deinitCompute;

    public:

      /** \brief Constructor that sets default values for member variables. */
      RTI ();

      /** \brief This destructor destroys the cloud, normals and search method used for
       * finding KNN. In other words it frees memory.
       */
      virtual
      ~RTI ();

      /** \brief Returns the number of nearest neighbours used for KNN. */
      unsigned int
      getNumberOfNeighbours () const;

      /** \brief Allows to set the number of neighbours. For more information check the article.
       * \param[in] neighbour_number number of neighbours to use
       */
      void
      setNumberOfNeighbours (unsigned int neighbour_number);

    protected:

      /** \brief Number of neighbours to find. */
      unsigned int neighbour_number_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#include <impl/RTI.hpp>




#endif /* RTI_H_ */
