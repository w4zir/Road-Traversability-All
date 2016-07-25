/*
 * road_detection.h
 *
 *  Created on: Oct 13, 2015
 *      Author: mudassir
 */

#ifndef ROAD_DETECTION_H_
#define ROAD_DETECTION_H_

#ifndef PCL_FILTERS_BILATERAL_H_
 #define PCL_FILTERS_BILATERAL_H_

 #include <pcl/filters/filter.h>

 namespace pcl
 {
   template<typename PointT>
   class BilateralFilter : public Filter<PointT>
   {
   };
 }

 #endif // PCL_FILTERS_BILATERAL_H_



#endif /* ROAD_DETECTION_H_ */
