#define PCL_NO_PRECOMPILE
#include <string>
#include <iostream>
#include <math.h>
#include <sys/time.h>
#include <vector>
#include <ctime>
#include <dirent.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/transforms.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/voxel_grid.h>
//#include <eigen3/Eigen/Array>

using namespace std;
using namespace pcl;

const float PI = 3.1415927;
struct PointXYZTP
{
    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    float theta;
    float phi;
    int valid;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZTP,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, theta, theta)
                                   (float, phi, phi)
                                   (int, valid, valid)
)
struct carRoadPos
{
    pcl::PointXYZ t1Pos; 
    pcl::PointXYZ t2Pos; 
    pcl::PointXYZ t3Pos; 
    pcl::PointXYZ t4Pos; 
    //    float theta; // front axle orientation
    //    float phi; // front axle orientation
};
struct carConf
{
    pcl::PointXYZ position; // front axle orientation
    float theta; // front axle orientation
    float phi; // front axle orientation
};
class RoadQuality {
  public:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ; 
    pcl::PointCloud<PointXYZTP>::Ptr prmCloud; 
    pcl::PointCloud<PointXYZRGB>::Ptr prmCloudRGB;
    pcl::ModelCoefficients::Ptr coefficients; // (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudProjected;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
    KdTreeFLANN<PointXYZTP> ftree;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *octree;
    PointXYZTP pointTP;

    const static float xMin = -2.0;

    const static float xMax = 2.0;
    const static float yMin = -3; 
    const static float yMax = 3;
    const static float zMin = 0;
    const static float zMax = 0;
    const static float thetaRange = 60;
    const static float phiRange = 0;

    const static double wbWidth = 1.3; //v1=1.4, v2=2, v3=2.5   for real road used 1.0
    const static double wbLength = 2.4; //v1=2.4, v2=3, v3=4    for real road used 1.6
    const static double vehWidth = 1.5; //in meter default 1.6
    const static double vehLength = 3.5; //in meter default 2.7
    const static double frontMaxAnlge = 30;
    const static double backMaxAnlge = 360;
    const static double wheelSurfaceRadius = 0.1;
    const static double wheelWidth = 0.2;
    const static double wheelRoadContactLength;
    const static double wheelAllowedHeight;
    const static double vehicleGroundClearance = 0.2;

    const static int wConfigCount = 40;
    const static int lConfigCount = 60;
    const static int tConfigCount = 1;

    pcl::PointXYZ minPts,maxPts;
    Eigen::Vector3f normalXYPlane;
    Eigen::MatrixXf vPosition;
    Eigen::MatrixXf vTirePosition;
    pcl::PointXYZRGB randPlanePoint;
    Eigen::MatrixXf adjMatrix;
    Eigen::MatrixXf degMatrix;
    Eigen::MatrixXf lapMatrix;
    double pCoeffMod;
    ///The z_min and z_max value for filter.
    const static double z_min;
    const static double z_max;

    // distance threshold value for plane segmentation using RANSAC
    const static double planeThreshold = 0.01; // for synthetic data use 0.01
    const static double planeDistThreshold = 0.1; //for synthetic data use 0.1
    const static double perValidThreshold = 0.9;
    const static int subConfigCount = 3;
    const static double nSearchRadiusFactor = 2;
    const static double negObsTh = 0.2;
    const static double posObsTh = 0.3;
  public:
    //Eigen::MatrixXf getVehicleConfigurations(Eigen::MatrixXf ,double ,double ,double ,double ,double ,bool);
    void getVehicleConfigurations();
    void getVehicleRandomConfigurations();
    Eigen::MatrixXf getVehicleTyresPosition(PointXYZTP);
    Eigen::MatrixXf getVehicleBoundingBox(PointXYZTP);
    Eigen::MatrixXf getVehicleRectangle(PointXYZTP);
    bool checkCollision(Eigen::MatrixXf ,double );
    void validateConfigs();
    bool checkCollision(Eigen::MatrixXf,Eigen::MatrixXf);
    void getPlaneCoefficients(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
    void getPlaneCoefficients(pcl::PointCloud<pcl::PointXYZ>::Ptr);
    void generatePRM();
    void generatePRM2();
    void transformCloud2XYPlane();
    void preProcessPointCloud();
    //RoadQuality();
    //void set_values (int,int);
    //int area() {return width*height;}
    // Member functions definitions including constructor
    RoadQuality(void) {
      cloudXYZ = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
      cloudProjected = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
      coefficients = pcl::ModelCoefficients::Ptr (new pcl::ModelCoefficients);
      tree = pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>);
      octree= (new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(128.0f));
      //ftree = KdTreeFLANN<PointXYZTP> (new KdTreeFLANN<PointXYZTP>);
      normalXYPlane =  Eigen::Vector3f(0.0,0.0,1.0);
      adjMatrix = Eigen::MatrixXf::Constant(wConfigCount*lConfigCount*tConfigCount,wConfigCount*lConfigCount*tConfigCount,-1);
      degMatrix = Eigen::MatrixXf::Zero(wConfigCount*lConfigCount*tConfigCount,wConfigCount*lConfigCount*tConfigCount);
      lapMatrix = Eigen::MatrixXf::Zero(wConfigCount*lConfigCount*tConfigCount,wConfigCount*lConfigCount*tConfigCount);
      vPosition = Eigen::MatrixXf(3,4);
      vPosition << -vehWidth/2,vehWidth/2,vehWidth/2,-vehWidth/2,-vehLength/2,-vehLength/2,vehLength/2,vehLength/2,0,0,0,0;
      //vPosition << -vehWidth/2,-vehWidth/2,-vehWidth/2,-vehWidth/2,vehWidth/2,vehWidth/2,vehWidth/2,vehWidth/2,-vehLength/2,-vehLength/2,vehLength/2,vehLength/2,-vehLength/2,-vehLength/2,vehLength/2,vehLength/2,-1,1,-1,1,-1,1,-1,1;
      vTirePosition = Eigen::MatrixXf(3,4);
      vTirePosition << -wbWidth/2,wbWidth/2,wbWidth/2,-wbWidth/2,-wbLength/2, -wbLength/2, wbLength/2,wbLength/2,0,0,0,0;
      //cout <<"end initialize normal and carPos\n";
    }
    void setPRMPoint(PointXYZTP point) {
      pointTP = point;
    }
    void setPRMCloud(pcl::PointCloud<PointXYZTP>::Ptr cloud) {
      prmCloud = cloud;
      prmCloud->height = 1;
      prmCloud->width = prmCloud->points.size();
    } 
    void setPRMRGBCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
      prmCloudRGB = cloud;
      prmCloudRGB->height = 1;
      prmCloudRGB->width = prmCloudRGB->points.size();
    }
    void setInCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
      inCloud = cloud;
      inCloud->height = 1;
      inCloud->width = inCloud->points.size();
    }
    void getXYZCloud() {
      copyPointCloud(*inCloud,*cloudXYZ);
    }	
    void projectCloudOnPlane() {
      pcl::ProjectInliers<pcl::PointXYZ> proj;
      proj.setModelType (pcl::SACMODEL_PLANE);
      proj.setInputCloud (cloudXYZ);
      proj.setModelCoefficients (coefficients);
      proj.filter (*cloudProjected);
    }
    void setTreeCloud() {
      //cout << "projected pointcloud size \t" << cloudProjected->points.size() <<endl;
      tree->setInputCloud (cloudProjected);
    }
    void setOcTreeCloud() {
      //cout << "projected pointcloud size \t" << cloudProjected->points.size() <<endl;
      octree->setInputCloud (cloudProjected);
      octree->addPointsFromInputCloud ();
    }
    void setfTreeCloud() {
      ftree.setInputCloud (prmCloud);
    }
    void getMinMax() {
      cout <<"min max of original pointcloud \n";
      pcl::getMinMax3D(*cloudXYZ,minPts,maxPts);
      std::cout << "minX:"<<minPts.x<<" minY:"<<minPts.y<<" minZ:"<<minPts.z<<std::endl;
      std::cout << "maxX:"<<maxPts.x<<" maxY:"<<maxPts.y<<" maxZ:"<<maxPts.z<<std::endl;
    }
    double pointDistFromLine(Eigen::Vector3f point, Eigen::MatrixXf point1, Eigen::MatrixXf point2) {
      Eigen::Vector3f d1=point-point1;
      Eigen::Vector3f d2=point-point2;
      Eigen::Vector3f d3=point2-point1;
      Eigen::Vector3f p = d1.cross(d2);
      return p.norm()/d3.norm();
    } 		
};
void RoadQuality::preProcessPointCloud() {
  // Create the filtering object
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (inCloud);
  sor.setLeafSize (0.1f, 0.1f, 0.1f);
  sor.filter (*cloud_filtered);
  inCloud = cloud_filtered;
  cout << "processed pointcloud has "<<inCloud->points.size()<<" points."<<endl;
  pcl::PCDWriter asd;
  asd.write<pcl::PointXYZRGB> ("test.pcd", *inCloud, false);
}
void RoadQuality::getPlaneCoefficients(pcl::PointCloud<PointXYZ>::Ptr cloud) {
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (planeThreshold);
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);
  pCoeffMod = sqrt(pow(coefficients->values[0],2)+pow(coefficients->values[1],2)+pow(coefficients->values[2],2));
  cout << "coefficients are "<<*coefficients<<endl;
}
void RoadQuality::getPlaneCoefficients(pcl::PointCloud<PointXYZRGB>::Ptr cloud) {
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (planeThreshold);
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);
  pCoeffMod = sqrt(pow(coefficients->values[0],2)+pow(coefficients->values[1],2)+pow(coefficients->values[2],2));
  cout << "coefficients are "<<*coefficients<<endl;
}
void RoadQuality::getVehicleConfigurations()
{
  cout << "enter function getVehicleCOnfigurations \n";	
  pcl::PointCloud<PointXYZTP>::Ptr configCloud (new pcl::PointCloud<PointXYZTP> ()); 	
  //pcl::PointCloud<PointXYZRGB>::Ptr configRGBCloud  (new pcl::PointCloud<PointXYZRGB> ());

  double roadAllowedWidth = (maxPts.x-minPts.x);
  double roadAllowedLength = (maxPts.y-minPts.y);
  cout <<"road allowed width: \t"<<roadAllowedWidth<<", length:\t"<<roadAllowedLength<<endl;
  //  double roadAllowedWidth = (xMax-xMin-wbWidth-wheelWidth);
  //  double roadAllowedLength = (yMax-yMin-wbLength-wheelWidth);

  double wInc = roadAllowedWidth/(wConfigCount-1); // because start and end are included
  double lInc = roadAllowedLength/(lConfigCount-1); // because start and end are included
  double tInc = 10;
  for (int i=0;i<lConfigCount;i++) {
    for (int j=0;j<wConfigCount;j++) {
      for (int k=0;k<tConfigCount;k++) {
      PointXYZTP point;
      point.x = minPts.x + j*wInc;
      point.y = minPts.y + i*lInc;
      //point.x = xMin + wbWidth/2 + wheelWidth/2 + j*wInc;
      //point.y = yMin + wbLength/2 + wheelWidth/2 + i*lInc;
      point.z = 0.0f;
      point.theta = -(thetaRange)/2+k*tInc;//(thetaRange) * rand () / (RAND_MAX + 1.0) - thetaRange/2;//0.0f;
      point.phi = 0.0f;
      point.valid = 1;
      configCloud->points.push_back(point);
      }
    }
  }
  setPRMCloud(configCloud);	
}
void RoadQuality::getVehicleRandomConfigurations()
{
  cout << "enter function getVehicleRandomConfigurations \n";
  pcl::PointCloud<PointXYZTP>::Ptr configCloud (new pcl::PointCloud<PointXYZTP> ());
  pcl::PointXYZRGB randPlanePoint;
  double randPlanePointDist = 0;
  do
  {
    int randNo = std::rand();// / (RAND_MAX + 1.0f);
    int cloudIdx = randNo%inCloud->size();
    randPlanePoint.x = inCloud->points[cloudIdx].x;
    randPlanePoint.y = inCloud->points[cloudIdx].y;
    randPlanePoint.z = inCloud->points[cloudIdx].z;
    randPlanePointDist = abs((coefficients->values[0]*randPlanePoint.x +
        coefficients->values[1]*randPlanePoint.y +
        coefficients->values[2]*randPlanePoint.z + coefficients->values[3]));
  }while(randPlanePointDist > planeThreshold);
  // std::cout <<"random point on plane has index:"<<cloudIdx<< "\tx:"<<randPlanePoint.x<<"\t y:"<<randPlanePoint.y<<"\t z:"<<randPlanePoint.z<<"\t rgb"<<randPlanePoint.rgb<<endl;
  std::cout <<"random point on plane has distance from plane:"<<randPlanePointDist<<endl;

  for (int i=0;i<lConfigCount;i++) {
    for (int j=0;j<wConfigCount;j++) {
      PointXYZTP point;
      // generate random x,y and z values
      float xVal = (maxPts.x-minPts.x) * rand () / (RAND_MAX + 1.0f) + minPts.x;
      float yVal = (maxPts.y-minPts.y) * rand () / (RAND_MAX + 1.0f) + minPts.y;
      float zVal = (maxPts.z-minPts.z) * rand () / (RAND_MAX + 1.0f) + minPts.z;
      // project the random x,y,z onto the plane
      Eigen::Vector3f normal (coefficients->values[0],coefficients->values[1],coefficients->values[2]);
      Eigen::Vector3f originPoint (randPlanePoint.x,randPlanePoint.y,randPlanePoint.z);
      Eigen::Vector3f randPoint (xVal,yVal,zVal);
      Eigen::Vector3f diffVector = randPoint-originPoint;
      // std::cout <<"diff vector values are:"<<diffVector(0)<<","<<diffVector(1)<<","<<diffVector(2)<<endl;
      float distFromPlane = diffVector.dot(normal);
      // std::cout <<"distance of point from plane:"<<distFromPlane<<endl;
      Eigen::Vector3f projectePoint = randPoint - distFromPlane*normal;
      //  std::cout <<"project point:"<<projectePoint(0)<<","<<projectePoint(1)<<","<<projectePoint(2)<<endl;
      point.x = projectePoint(0);
      point.y = projectePoint(1);
      point.z = projectePoint(2);
      point.theta = (thetaRange) * rand () / (RAND_MAX + 1.0) - thetaRange/2;
      point.phi = 0;// (phiRange) * rand () / (RAND_MAX + 1.0) - phiRange/2;
      point.valid = 1;
      configCloud->points.push_back(point);
    }
  }
  setPRMCloud(configCloud);
}
void RoadQuality::transformCloud2XYPlane() {
  //get transformation
  Eigen::Vector3f normalXYPlane (0.0,0.0,1.0);
  Eigen::Vector3f normalRoadPlane (coefficients->values[0],coefficients->values[1],coefficients->values[2]);
  Eigen::Vector3f cNormal = normalRoadPlane.cross(normalXYPlane);
  float sNormal = sqrt(cNormal.squaredNorm());
  float dNormal = normalRoadPlane.dot(normalXYPlane);
  Eigen::Matrix3f cNormalSkewSymm;
  cNormalSkewSymm << 0, -cNormal(2), cNormal(1), cNormal(2), 0, -cNormal(0), -cNormal(1), cNormal(0), 0;
  Eigen::Matrix3f rot2XYPlane;
  rot2XYPlane = Eigen::Matrix3f::Identity(3,3)+cNormalSkewSymm+cNormalSkewSymm*cNormalSkewSymm*((1-dNormal)/pow(sNormal,2));
  /*  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block(0,0,3,3) =  rot2XYPlane;*/
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << 0, 0.0, coefficients->values[3];
  transform.rotate (rot2XYPlane);
  pcl::transformPointCloud (*cloudXYZ, *cloudXYZ, transform);
  cout <<"points in transformed pointclou:\t"<<cloudXYZ->points.size()<<endl;
}

Eigen::MatrixXf RoadQuality::getVehicleTyresPosition(PointXYZTP vConfig)
{
  /*
   *  Find the vehicle wheels position on the road plane
   */
  Eigen::Vector3f pointOffset(vConfig.x,vConfig.y,vConfig.z);
  Eigen::MatrixXf positionOffset(3,4);
  positionOffset << pointOffset, pointOffset, pointOffset, pointOffset;
  Eigen::AngleAxis<float> trans((PI*vConfig.theta)/180,normalXYPlane);
  Eigen::Matrix3f tranMatrix;
  tranMatrix = trans.matrix();
  Eigen::MatrixXf xyTranformed(3,4);
  xyTranformed = tranMatrix*vTirePosition;
  /*  Eigen::MatrixXf rotPoint2RoadNormal(3,4);
      rotPoint2RoadNormal = rot2RoadPlane*xyTranformed;
      Eigen::MatrixXf carPositionRoad(3,4);
      carPositionRoad = rotPoint2RoadNormal+positionOffset;*/
  // for synthetic data
  Eigen::MatrixXf wbPositionRoad(3,4);
  wbPositionRoad = xyTranformed+positionOffset;
  return wbPositionRoad;
}
Eigen::MatrixXf RoadQuality::getVehicleBoundingBox(PointXYZTP vConfig)
{
  /*
   *  Find the bounding box that encloses vehicle on the road plane
   */
  Eigen::Vector3f pointOffset(vConfig.x,vConfig.y,vConfig.z);
  Eigen::MatrixXf positionOffset(3,8);
  positionOffset << pointOffset, pointOffset, pointOffset, pointOffset, pointOffset, pointOffset, pointOffset, pointOffset;
  Eigen::AngleAxis<float> trans((PI*vConfig.theta)/180,normalXYPlane);
  Eigen::Matrix3f tranMatrix;
  tranMatrix = trans.matrix();
  Eigen::MatrixXf xyTranformed(3,8);
  xyTranformed = tranMatrix*vPosition;
  /*  Eigen::MatrixXf rotPoint2RoadNormal(3,4);
      rotPoint2RoadNormal = rot2RoadPlane*xyTranformed;
      Eigen::MatrixXf carPositionRoad(3,4);
      carPositionRoad = rotPoint2RoadNormal+positionOffset;*/
  // for synthetic data
  Eigen::MatrixXf carPositionRoad(3,8);
  carPositionRoad = xyTranformed+positionOffset;
  return carPositionRoad;
}
Eigen::MatrixXf RoadQuality::getVehicleRectangle(PointXYZTP vConfig)
{
  /*
   *  Find the bounding box that encloses vehicle on the road plane
   */
  Eigen::Vector3f pointOffset(vConfig.x,vConfig.y,vConfig.z);
  Eigen::MatrixXf positionOffset(3,4);
  positionOffset << pointOffset, pointOffset, pointOffset, pointOffset;
  Eigen::AngleAxis<float> trans((PI*vConfig.theta)/180,normalXYPlane);
  Eigen::Matrix3f tranMatrix;
  tranMatrix = trans.matrix();
  Eigen::MatrixXf xyTranformed(3,4);
  xyTranformed = tranMatrix*vPosition;
  /*  Eigen::MatrixXf rotPoint2RoadNormal(3,4);
      rotPoint2RoadNormal = rot2RoadPlane*xyTranformed;
      Eigen::MatrixXf carPositionRoad(3,4);
      carPositionRoad = rotPoint2RoadNormal+positionOffset;*/
  // for synthetic data
  Eigen::MatrixXf carPositionRoad(3,4);
  carPositionRoad = xyTranformed+positionOffset;
  return carPositionRoad;
}
void RoadQuality::generatePRM2() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr prmCloudXYZ (new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(*prmCloudRGB,*prmCloudXYZ);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr prmtree (new pcl::search::KdTree<pcl::PointXYZ>);
    prmtree->setInputCloud (prmCloudXYZ);
    std::vector<int> cloudIdx;
    pcl::ExtractIndices<PointXYZTP> extIdx;
    extIdx.setInputCloud(prmCloud);
    extIdx.filter(cloudIdx);

    for (size_t it=0;it<cloudIdx.size();++it) {
      //cout <<"selected point valid flag:\t"<<prmCloud->points[cloudIdx[it]].valid<<endl;
      if(prmCloud->points[cloudIdx[it]].valid == 0) {
        //cout <<"selected configuration is invalid\n";
        adjMatrix.col(cloudIdx[it]) << Eigen::MatrixXf::Constant(lConfigCount*wConfigCount,1,0);
        continue;
      } else {
        PointXYZ point;
        point.x = prmCloud->points[cloudIdx[it]].x;
        point.y = prmCloud->points[cloudIdx[it]].y;
        point.z = prmCloud->points[cloudIdx[it]].z;
        //point.theta = prmCloud->points[cloudIdx[it]].theta;
        //point.phi = prmCloud->points[cloudIdx[it]].phi;

        std::vector<int> pointIdx;
        std::vector<float> pointDist;
        //ftree.radiusSearch(point,nSearchRadiusFactor*wheelWidth,pointIdx,pointDist,0);
        prmtree->radiusSearch(point,nSearchRadiusFactor*wheelWidth,pointIdx,pointDist);
        //ftree.nearestKSearch(point,28,pointIdx,pointDist);
        if(pointIdx.size()==0) {
          //cout <<"no neighbor with search radius "<<endl;
          adjMatrix.col(cloudIdx[it]) << Eigen::MatrixXf::Constant(lConfigCount*wConfigCount,1,0);
          continue;
        }
        int nCount = 0;
        for (size_t i=0;i<pointIdx.size();++i) {
          if(prmCloud->points[pointIdx[i]].valid==0) {
            //cout <<"neighbor configuration is invalid\n";
            adjMatrix(cloudIdx[it],pointIdx[i]) = 0;
            continue;
          }
          if(cloudIdx[it] == pointIdx[i]) {
            //cout <<"neighbor is same as selected configuration \n";
            adjMatrix(cloudIdx[it],pointIdx[i]) = 0;
            continue;
          }
          if(adjMatrix(cloudIdx[it],pointIdx[i])!=-1) {
                      //cout <<"neighbor is same as selected configuration \n";
                      continue;
           }
          double xInc = (point.x - prmCloud->points[pointIdx[i]].x)/subConfigCount;
          double yInc = (point.y - prmCloud->points[pointIdx[i]].y)/subConfigCount;
          double zInc = (point.z - prmCloud->points[pointIdx[i]].z)/subConfigCount;
          double thetaInc = (prmCloud->points[cloudIdx[it]].theta - prmCloud->points[pointIdx[i]].theta)/subConfigCount;
          double phiInc = (prmCloud->points[cloudIdx[it]].phi - prmCloud->points[pointIdx[i]].phi)/subConfigCount;
          // check sub points between two neibouring nodes for validity
          for (size_t j=1;j<subConfigCount;++j) {
            PointXYZTP subPoint;
            subPoint.x = point.x+j*xInc;
            subPoint.y = point.y+j*yInc;
            subPoint.z = point.z+j*zInc;
            subPoint.theta = prmCloud->points[cloudIdx[it]].theta+j*thetaInc;
            subPoint.phi = prmCloud->points[cloudIdx[it]].phi+j*phiInc;
            Eigen::MatrixXf vTyrePos = getVehicleTyresPosition(subPoint);
            Eigen::MatrixXf vBoundingBox = getVehicleRectangle(subPoint);
            bool isValid = checkCollision(vTyrePos,vBoundingBox);
            if(!isValid) {
              //cout <<"sub configuration is invalid\n";
              break;
            }
            if(j==(subConfigCount-1)) {
              //cout <<"found a valid neighbor\n";
              adjMatrix(cloudIdx[it],pointIdx[i]) = 1;
              adjMatrix(pointIdx[i],cloudIdx[it]) = 1;
              nCount++;
            }
          }
        }
        degMatrix(cloudIdx[it],cloudIdx[it]) = nCount;
      }
    }
    lapMatrix = degMatrix - adjMatrix;
}
void RoadQuality::generatePRM() {
  std::vector<int> cloudIdx;
  pcl::ExtractIndices<PointXYZTP> extIdx;
  extIdx.setInputCloud(prmCloud);
  extIdx.filter(cloudIdx);
  for (size_t it=0;it<cloudIdx.size();++it) {
    //cout <<"selected point valid flag:\t"<<prmCloud->points[cloudIdx[it]].valid<<endl;
    if(prmCloud->points[cloudIdx[it]].valid == 0) {
      //cout <<"selected configuration is invalid\n";
      continue;
    } else {
      PointXYZTP point;
      point.x = prmCloud->points[cloudIdx[it]].x;
      point.y = prmCloud->points[cloudIdx[it]].y;
      point.z = prmCloud->points[cloudIdx[it]].z;
      point.theta = prmCloud->points[cloudIdx[it]].theta;
      point.phi = prmCloud->points[cloudIdx[it]].phi;

      std::vector<int> pointIdx;
      std::vector<float> pointDist;
      //ftree.radiusSearch(point,nSearchRadiusFactor*wheelWidth,pointIdx,pointDist,0);
      ftree.nearestKSearch(point,28,pointIdx,pointDist);
      if(pointIdx.size()==0) {
        //cout <<"no neighbor with search radius "<<endl;
        continue;
      }
      int nCount = 0;
      for (size_t i=0;i<pointIdx.size();++i) {
        if(prmCloud->points[pointIdx[i]].valid==0) {
          //cout <<"neighbor configuration is invalid\n";
          continue;
        }
        if(cloudIdx[it] == pointIdx[i]) {
          //cout <<"neighbor is same as selected configuration \n";
          continue;
        }
        double xInc = (point.x - prmCloud->points[pointIdx[i]].x)/subConfigCount;
        double yInc = (point.y - prmCloud->points[pointIdx[i]].y)/subConfigCount;
        double zInc = (point.z - prmCloud->points[pointIdx[i]].z)/subConfigCount;
        double thetaInc = (point.theta - prmCloud->points[pointIdx[i]].theta)/subConfigCount;
        double phiInc = (point.phi - prmCloud->points[pointIdx[i]].phi)/subConfigCount;
        // check sub points between two neibouring nodes for validity
        for (size_t j=1;j<subConfigCount;++j) {
          PointXYZTP subPoint;
          subPoint.x = point.x+j*xInc;
          subPoint.y = point.y+j*yInc;
          subPoint.z = point.z+j*zInc;
          subPoint.theta = point.theta+j*thetaInc;
          subPoint.phi = point.phi+j*phiInc;
          Eigen::MatrixXf vTyrePos = getVehicleTyresPosition(subPoint);
          Eigen::MatrixXf vBoundingBox = getVehicleRectangle(subPoint);
          bool isValid = checkCollision(vTyrePos,vBoundingBox);
          if(!isValid) {
            //cout <<"sub configuration is invalid\n";
            break;
          }
          if(j==(subConfigCount-1)) {
            //cout <<"found a valid neighbor\n";
            adjMatrix(cloudIdx[it],pointIdx[i]) = 1;
            adjMatrix(pointIdx[i],cloudIdx[it]) = 1;
            nCount++;
          }
        }
      }
      degMatrix(cloudIdx[it],cloudIdx[it]) = nCount;
    }
  }
  lapMatrix = degMatrix - adjMatrix;
}
void RoadQuality::validateConfigs() {
  int validCount = 0;
  int invalidCount = 0;
  // int counter = 1;
  pcl::PointCloud<PointXYZRGB>::Ptr configRGBCloud  (new pcl::PointCloud<PointXYZRGB> ());
  for (pcl::PointCloud<PointXYZTP>::iterator it = prmCloud->points.begin (); it != prmCloud->points.end (); ++it) {
    // cout <<"prm counter:"<<counter<<endl;
    // counter++;
    PointXYZTP point;
    point.x = it->x;
    point.y = it->y;
    point.z = it->z;
    point.theta = it->theta;
    point.phi = it->phi;
    point.valid = it->valid;
    Eigen::MatrixXf vTyrePos = getVehicleTyresPosition(point);
    Eigen::MatrixXf vBoundingBox = getVehicleRectangle(point);
    //Eigen::MatrixXf vBoundingBox = getVehicleBoundingBox(point);
    bool isValid = checkCollision(vTyrePos,vBoundingBox);

    pcl::PointXYZRGB pointRGB;
    pointRGB.x = it->x;
    pointRGB.y = it->y;
    pointRGB.z = it->z;
    pointRGB.r = 0;
    pointRGB.b = 0;
    pointRGB.g = 0;
    if(isValid) {
      it->valid = 1;
      pointRGB.g = 255;
      validCount++;
    } else {
      it->valid = 0;
      pointRGB.r = 255;
      invalidCount++;
    }
    configRGBCloud->points.push_back(pointRGB);
  }
  setPRMRGBCloud(configRGBCloud);
  cout <<"valid:\t"<<validCount<<"\t invalid:\t"<<invalidCount<<endl;
}

bool RoadQuality::checkCollision(Eigen::MatrixXf tyrePos, Eigen::MatrixXf vehicleBox)
{
  //double cpu5  = clock() / CLOCKS_PER_SEC;
  // bool isValid = true;
  for(size_t i=0;i<4;i++) {
    if(tyrePos(0,i)<minPts.x || tyrePos(0,i)>maxPts.x) {
      return false;
    }
  }

  for (size_t i=0;i<4;i++) {
    pcl::PointXYZ tyrePoint;
    tyrePoint.x = tyrePos(0,i);
    tyrePoint.y = tyrePos(1,i);
    tyrePoint.z = tyrePos(2,i);
    //cout <<"tyre position \t"<<tyrePos <<endl;
    /*
     * Find the points in the project cloud around the tire centre
     */
    std::vector<int> pointIdx;
    std::vector<float> pointDist;
    tree->radiusSearch(tyrePoint,wheelWidth/2,pointIdx,pointDist,0);

    //cout <<"points under vehicle tyre \t"<<pointIdx.size() <<endl;
    if(pointIdx.size()<2) // not enough points udner tyre
    {
      //isValid = false;
      // break;
      continue;
    }

    // enoughPoints++;
    //pcl::PointXYZ highPoint;
    double maxDist = 0;
    double validPoint = 0;
    double invalidPoint = 0;
    //     for (pcl::PointCloud<pcl::PointXYZ>::iterator it = tyreBoxCloud->points.begin (); it != tyreBoxCloud->points.end (); ++it)
    for (size_t k=0;k<pointIdx.size();k++)
    {
      //        double distFromPlane = abs((coefficients->values[0]*it->x + coefficients->values[1]*it->y + coefficients->values[2]*it->z + coefficients->values[3])/
      //                                   (coeffSqrt));
      double distFromPlane = ((coefficients->values[0]*cloudXYZ->points[pointIdx[k]].x +
          coefficients->values[1]*cloudXYZ->points[pointIdx[k]].y +
          coefficients->values[2]*cloudXYZ->points[pointIdx[k]].z + coefficients->values[3]))/
              (pCoeffMod);
      // std::cout <<"distance from plane:"<< distFromPlane << endl;
      if(distFromPlane<-negObsTh || distFromPlane > posObsTh)
      {
        invalidPoint++;
      }else
      {
        validPoint++;
      }
    }
    double validRatio = validPoint/(validPoint+invalidPoint);
    //   std::cout << "neighboring point size:"<<pointIdx.size()<<"\t valid points:"<<validPoint<<"\t invalid points:"<<invalidPoint<<"\t valid points ratio:"<<validRatio<<endl;

    if(validRatio < perValidThreshold)
    {
      return false;
    }
  }

  /*
   * Hull method
   */
  /* 	pcl::PointCloud<pcl::PointXYZ>::Ptr vehicleBoundingBox (new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i=0;i<8;i++) {
	 vehicleBoundingBox->push_back(pcl::PointXYZ(vehicleBox(0,i), vehicleBox(1,i), vehicleBox(2,i)));
	}
double cpu5  = clock() / CLOCKS_PER_SEC;
  	pcl::ConvexHull<pcl::PointXYZ> hull;
  	hull.setInputCloud(vehicleBoundingBox);
	  hull.setDimension(3);
	  std::vector<pcl::Vertices> polygons;

	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
	  hull.reconstruct(*cloud_hull, polygons);

	pcl::PointCloud<pcl::PointXYZ>::Ptr vBox (new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::CropHull<pcl::PointXYZ> bb_filter;

	  bb_filter.setDim(3);
	  bb_filter.setInputCloud(cloudXYZ);
	  bb_filter.setHullIndices(polygons);
	  bb_filter.setHullCloud(cloud_hull);
	  bb_filter.filter(*vBox);
	  cout << "hull has points:"<<vBox->points.size()<<endl;
	  for (pcl::PointCloud<PointXYZ>::iterator it = vBox->points.begin (); it != vBox->points.end (); ++it) {
	    double distFromPlane = (coefficients->values[0]*it->x + coefficients->values[1]*it->y + coefficients->values[2]*it->z + coefficients->values[3])/(pCoeffMod);
	    if (distFromPlane > vehicleGroundClearance) {
	      return false;
	    }
	  }
	double cpu6  = clock() / CLOCKS_PER_SEC;
	cout <<"time for hull=\t"<<cpu6-cpu5<<endl;
   */
  /*
   * Box Search
   */
  /*  double cpu5  = clock() / CLOCKS_PER_SEC;
  std::vector<int> pointIdx;
  octree->boxSearch (vehicleBox.col(0),vehicleBox.col(7),pointIdx);
  //cout <<"start of box:"<<vehicleBox.col(0)<<"\n end of box:"<<vehicleBox.col(7)<<endl;
  //cout << "point inside box are :"<<pointIdx.size()<<endl;
  for (size_t i=0;i<pointIdx.size();i++) {
    double distFromPlane = ((coefficients->values[0]*cloudXYZ->points[pointIdx[i]].x +
        coefficients->values[1]*cloudXYZ->points[pointIdx[i]].y +
        coefficients->values[2]*cloudXYZ->points[pointIdx[i]].z + coefficients->values[3]))/
            (pCoeffMod);
    if (distFromPlane > vehicleGroundClearance) {
      return false;
    }
  }*/

  /*
   * triangle method
   */
  // double cpu5  = clock() / CLOCKS_PER_SEC;
  int counter =0;
  for (size_t i=0;i<cloudProjected->points.size();i++) {
    //cout <<"point is :"<<cloud_projected->points[i].x<<","<<cloud_projected->points[i].y<<","<<cloud_projected->points[i].z<<endl;
    //    Eigen::Vector3f confPoint(it->x,it->y,it->z);
    Eigen::Vector3f confPoint(cloudProjected->points[i].x,cloudProjected->points[i].y,cloudProjected->points[i].z);
    double d1 = pointDistFromLine(confPoint,vehicleBox.col(0),vehicleBox.col(1));
    double d2 = pointDistFromLine(confPoint,vehicleBox.col(1),vehicleBox.col(2));
    double d3 = pointDistFromLine(confPoint,vehicleBox.col(2),vehicleBox.col(3));
    double d4 = pointDistFromLine(confPoint,vehicleBox.col(3),vehicleBox.col(0));
    //cout <<"distance is:"<<d1<<","<<d2<<","<<d3<<","<<d4<<endl;
    if (d1 < vehLength && d2 < vehWidth && d3 < vehLength &&  d4 < vehWidth) {
      counter++;
      double distFromPlane = ((coefficients->values[0]*inCloud->points[i].x + coefficients->values[1]*inCloud->points[i].y + coefficients->values[2]*inCloud->points[i].z + coefficients->values[3])/(pCoeffMod));
      if (distFromPlane > vehicleGroundClearance) {
        return false;
      }
    }
  }
  // double cpu6  = clock() / CLOCKS_PER_SEC;
  // 	cout <<"points in rectangle:"<<counter<<endl;
  // 	cout <<"time for triangle search=\t"<<cpu6-cpu5<<endl;
  return true;
}

double get_wall_time(){
  struct timeval time;
  if (gettimeofday(&time,NULL)){
    return 0;
  }
  return  time.tv_sec + time.tv_usec * .000001;
}
double get_cpu_time(){
  return clock() / CLOCKS_PER_SEC;
}
int
main (int argc, char *argv[])
{


  //  Start Timers
  double wall0 = get_wall_time();
  double cpu0  = get_cpu_time();
  srand(time(0));

  /*

if (argc < 11)
  {
    cerr << "usage: " << argv[0] << " input_file dist_from_plane planeDistThreshold perc_valid_points vehWidth vehLength wheel_width wheel_length_with_road wheel_allowed_height output_filename prm_op_file" << endl;
    exit (EXIT_FAILURE);
  }
  string fileName = argv[1];
  istringstream (argv[2]) >> planeThreshold;
  istringstream (argv[3]) >> planeDistThreshold;
  istringstream (argv[4]) >> perValidPoints;
  istringstream (argv[5]) >> vehWidth;
  istringstream (argv[6]) >> vehLength;
  istringstream (argv[7]) >> wheelWidth;
  istringstream (argv[8]) >> wheelRoadContactLength;
  istringstream (argv[9]) >> wheelAllowedHeight;
  string fileOutputName = argv[10];
  string filePRMOutputName = argv[11];
   */
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PCDWriter writer;

  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir ("/home/mudassir/phd_ws/data/road_synthetic_data/pointclouds/")) != NULL) {
    while ((ent = readdir (dir)) != NULL) {
      string fName = ent->d_name;
      if(fName.find("road_obs")!=std::string::npos)
      {
        double cpu1  = get_cpu_time();
        std::cout << endl<<"reading file -------------------------------------\t"<<fName.c_str() <<endl;
        std::stringstream readFile;
        // readFile << "/home/mudassir/phd_ws/data/road_synthetic_data/pointclouds/"<<fileName.c_str() << ".pcd";
        readFile << "/home/mudassir/phd_ws/data/road_synthetic_data/pointclouds/"<<fName.substr(0,fName.find(".pcd")) << ".pcd";
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        reader.read (readFile.str(), *cloud);

        RoadQuality obj;
        obj.setInCloud(cloud);  // set input cloud to obj.inCloud
        obj.preProcessPointCloud(); // downsample pointcloud to save computation time
        obj.getXYZCloud();
        obj.getPlaneCoefficients(obj.inCloud);
        // obj.transformCloud2XYPlane(); // not used for synthetic data on xy-plane
        obj.getMinMax();
        //obj.getVehicleConfigurations(); // generate all vehicle configurations
        obj.getPlaneCoefficients(obj.cloudXYZ);
        obj.getVehicleRandomConfigurations(); // generate random vehicle configurations
        obj.projectCloudOnPlane();
        obj.setTreeCloud();
        obj.setOcTreeCloud();
        cout <<"check configurations validity \n";
        obj.validateConfigs();
        obj.setfTreeCloud();
        cout <<"generating PRM graph \n";
        obj.generatePRM2();

        std::stringstream saveAdj;
        saveAdj <<"/home/mudassir/phd_ws/data/road_synthetic_data/adjacency_and_degree/v6/" << fName.substr(0,fName.find(".pcd"))<<"_adj";
        std::ofstream saveAdjFile(saveAdj.str().c_str());
        if (saveAdjFile.is_open())
        {
          saveAdjFile << obj.adjMatrix << "\n";
          saveAdjFile.close();
        }

        std::stringstream saveDeg;
        saveDeg <<"/home/mudassir/phd_ws/data/road_synthetic_data/adjacency_and_degree/v6/" << fName.substr(0,fName.find(".pcd"))<<"_deg";
        std::ofstream saveDegFile(saveDeg.str().c_str());
        if (saveDegFile.is_open())
        {
          saveDegFile << obj.degMatrix.diagonal() << "\n";
          saveDegFile.close();
        }

        std::stringstream savePRMFile;
        std::stringstream savePRMViewFile;
        savePRMViewFile << "/home/mudassir/phd_ws/data/road_synthetic_data/prm_synthetic_data/v6/"<< fName.substr(0,fName.find(".pcd")) << "_prm_view.pcd";
        savePRMFile << "/home/mudassir/phd_ws/data/road_synthetic_data/prm_synthetic_data/v6/"<< fName.substr(0,fName.find(".pcd")) << "_prm.pcd";
        writer.write<pcl::PointXYZRGB> (savePRMViewFile.str(), *obj.prmCloudRGB, false); //*
        writer.write<PointXYZTP> (savePRMFile.str(), *obj.prmCloud, false); //*
        double cpu2  = get_cpu_time();
        cout << "CPU Time after prm complete = " << cpu2  - cpu1  << endl;
        cout << "file written successfully" << endl<<endl;

      }
    }
    closedir (dir);
  } else {
    perror ("could not open directory");
    return EXIT_FAILURE;
  }
  //  writer.write<pcl::PointXYZRGB> (fileOutputName.c_str(), *prmCloudRGB, false); //*
  //  writer.write<PointXYZTP> (filePRMOutputName.c_str(), *prmCloud, false); //*
  //   pcl::visualization::CloudViewer viewer ("cloud_viewer");
  //   viewer.showCloud(prmCloudRGB);
  //   while(!viewer.wasStopped())
  //    {
  //    } */
  return (0);
}

