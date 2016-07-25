#include "include/road_segmentation.h"
#include "include/height_estimate.h"
//#include "include/RTI.h"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
  pcl::PCDWriter writer;
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);

  reader.read ("road_0.pcd", *cloud_in);
  std::cerr << "Points in cloud before filtering: " << cloud_in->points.size() << std::endl;


  pcl::search::Search<pcl::PointXYZ>::Ptr treeNorm = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud_in);
  ne.setSearchMethod (treeNorm);
  ne.setKSearch (50);
  ne.compute (*normals);

  std::cerr << "Points in normal cloud: " << normals->points.size() << std::endl;

  /************************************************************************************************************************************************************************
   * Segment road portion through passthrough filter
   * *********************************************************************************************************************************************************************** */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough_road (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_in);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (4.0, 6.0);
  pass.filter (*cloud_passthrough_road);
  pass.setInputCloud (cloud_passthrough_road);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-2.0, 2.0);
  pass.filter (*cloud_passthrough_road);

  pcl::SACSegmentation<pcl::PointXYZ> seg_pass;
  pcl::ModelCoefficients::Ptr model_coeff_pass (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_pass (new pcl::PointIndices);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
  seg_pass.setOptimizeCoefficients (true);
  seg_pass.setModelType (pcl::SACMODEL_PLANE);
  seg_pass.setMethodType (pcl::SAC_RANSAC);
  seg_pass.setMaxIterations (100);
  seg_pass.setDistanceThreshold (0.01);
  seg_pass.setInputCloud (cloud_passthrough_road);
  seg_pass.segment (*inliers_pass, *model_coeff_pass);
  std::cout << "coefficients are "<<*model_coeff_pass<<std::endl;
  double coeffSqrt_pass = std::sqrt(std::pow(model_coeff_pass->values[0],2)+std::pow(model_coeff_pass->values[1],2)+std::pow(model_coeff_pass->values[2],2));


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_height (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::HeightEstimate<pcl::PointXYZ> hEst;
  hEst.setInputCloud(cloud_in);
  hEst.setNeighboursRadius(0.2);
  hEst.setRoadPlaneCoefficients(model_coeff_pass);
  hEst.estimate(*cloud_height);
  hEst.getProjectedCloud(cloud_projected);
 // std::cerr << "Cloud after project has points: " << cloud_projected->points.size() << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr road_segment (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> point_labels;
  pcl::RoadSegmentation<pcl::PointXYZ,pcl::Normal> seg;
  seg.setInputCloud(cloud_height);
  seg.setNumberOfNeighbours (20);
  seg.setInputNormals (normals);
  seg.setRoadPlaneCoefficients(model_coeff_pass);
  //seg.setRoadNormal(Eigen::Vector3f(model_coeff_pass->values[0],model_coeff_pass->values[1],model_coeff_pass->values[2]));
  seg.setSmoothnessThreshold (5.0 / 180.0 * M_PI);
  seg.setCurvatureThreshold (1.0);
  seg.setOctreeResolution(0.1);
  seg.setProjectedCloud(cloud_projected);
  seg.detect(point_labels);
  seg.segment(*road_segment);


  int rCount=0;
  int nrCount=0;
  for (int i=0; i < point_labels.size(); i++)
  {
    if(point_labels[i]==1)
    {
      rCount++;
    }else if(point_labels[i]==-1)
    {
      nrCount++;
    }
  }
  std::cout << "total point:" << point_labels.size() << "\t road:" << rCount << "\t non-road:" << nrCount << std::endl;

 pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();
//  writer.write<pcl::PointXYZRGB> ("road_rgb.pcd", *colored_cloud, false);

  pcl::visualization::CloudViewer viewer ("Cluster viewer");

    viewer.showCloud(colored_cloud);
    while (!viewer.wasStopped ())
    {
    }

  return 0;
}
