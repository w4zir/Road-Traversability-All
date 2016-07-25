
#define PCL_NO_PRECOMPILE
#include <string>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Eigenvalues>

using namespace std;
using namespace pcl;

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

int main (int argc, char *argv[])
{
  // car specifications
  double carWidth; //in meter default 1.6
  double carLength; //in meter default 2.7
  double frontMaxAnlge = 30;
  double backMaxAnlge = 30;
  double wheelSurfaceRadius = 0.1;

  //srand(time(0));
  const float PI = 3.1415927;

  int sIdx;
  int eIdx;
  double xWeight;
  double yWeight;
  double zWeight;
  double phiWeight;
  double thetaWeight;
  double searchRadius;
  double thresholdDist;
  if (argc < 10)
  {
    cerr << "usage: " << argv[0] << " input_file sIdx eIdx xWeight yWeight zWeight phiWeight thetaWeight searchRadius thresholdDist" << endl;
    exit (EXIT_FAILURE);
  }
  string inputFile = argv[1];
  //  string axisName = argv[2];
  istringstream (argv[2]) >> sIdx;
  istringstream (argv[3]) >> eIdx;
  istringstream (argv[4]) >> xWeight;
  istringstream (argv[5]) >> yWeight;
  istringstream (argv[6]) >> zWeight;
  istringstream (argv[7]) >> phiWeight;
  istringstream (argv[8]) >> thetaWeight;
  istringstream (argv[9]) >> searchRadius;
  istringstream (argv[10]) >> thresholdDist;
  pcl::PCDReader reader;
  pcl::PCDWriter writer;
  //reader.read ("table_scene_lms400.pcd", *cloud);


  for (size_t dInx=1;dInx<=4;dInx++ )
  {
//    std::stringstream saveEigenMeanFile;
//    saveEigenMeanFile <<"/home/mudassir/phd_ws/data/RSS2015_data/eigen_values/" << inputFile.c_str() <<dInx;
//  std::ofstream eigfile;

  for (size_t idx=sIdx;idx<=eIdx;++idx)
  {
    pcl::PointCloud<PointXYZTP>::Ptr cloud (new pcl::PointCloud<PointXYZTP>);
    pcl::PointCloud<PointXYZTP>::Ptr cloud_valid (new pcl::PointCloud<PointXYZTP>);

    std::cout << endl<<"reading file "<<inputFile.c_str() <<dInx<< "_" << idx << "_prm.pcd"<<endl;
    std::stringstream readFile;
    readFile << "/home/mudassir/phd_ws/data/RSS2015_data/prm_type2/"<<inputFile.c_str()<<dInx << "_" << idx << "_prm.pcd";
    reader.read (readFile.str(), *cloud);

    std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

    for (pcl::PointCloud<PointXYZTP>::iterator it = cloud->points.begin (); it != cloud->points.end (); ++it)
    {
      //cout <<"valid flag "<<it->valid<<endl;
      if(it->valid==1)
      {
        PointXYZTP pointValid;
        pointValid.x = it->x;
        pointValid.y = it->y;
        pointValid.z = it->z;
        pointValid.theta =it->theta;
        pointValid.phi =it->phi;
        pointValid.valid =it->valid;
        cloud_valid->push_back(pointValid);
      }
    }
    cloud_valid->width = cloud_valid->points.size ();
    cloud_valid->height = 1;
    cloud_valid->is_dense = false;
    int validCloudSize = cloud_valid->points.size ();

    cout << "valid configuration count:"<<validCloudSize<<endl;
    if(validCloudSize<1)
    {
//          eigfile.open(saveEigenMeanFile.str().c_str() ,ios::app);
//          eigfile << 0 << '\n';
//          eigfile.close();
          //      }

      std::stringstream saveFile;
          saveFile <<"/home/mudassir/phd_ws/data/RSS2015_data/eigen_values/" << inputFile.c_str() <<dInx<< "_" << idx <<"_adj";
          std::ofstream savEigfile(saveFile.str().c_str());
          if (savEigfile.is_open())
          {
            savEigfile << 0 << 0 << "\n" << 0 << 0 << "\n";
            savEigfile.close();
          }

          std::stringstream saveDeg;
          saveDeg <<"/home/mudassir/phd_ws/data/RSS2015_data/eigen_values/" << inputFile.c_str() <<dInx<< "_" << idx <<"_deg";
              std::ofstream savDegfile(saveDeg.str().c_str());
              if (savDegfile.is_open())
              {
                savDegfile << 0 << "\n" << 0 << "\n";
                savDegfile.close();
              }
      continue;
    }

    std::vector<int> cloudIdx;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    pcl::ExtractIndices<PointXYZTP> extIdx;
    extIdx.setInputCloud(cloud_valid);
    extIdx.filter(cloudIdx);
    std::cout << "point indices size:"<<cloudIdx.size()<<endl;
    //  extIdx.getIndices();
    //  extIdx.ExtractIndices();
    // pcl::PointXYZRGB minPts,maxPts;
    //  pcl::getMinMax3D(*cloud,minPts,maxPts);
    //   std::cout << "minX:"<<minPts.x<<" minY:"<<minPts.y<<" minZ:"<<minPts.z<<" minRGB:"<<minPts.a<<endl;
    //   std::cout << "maxX:"<<maxPts.x<<" maxY:"<<maxPts.y<<" maxZ:"<<maxPts.z<<" maxRGB:"<<maxPts.rgba<<endl;

    // degine adjacency matrix for PRM graph
    Eigen::MatrixXf confAdjacencyMatrix;
    confAdjacencyMatrix = Eigen::MatrixXf::Constant(validCloudSize,validCloudSize,0);
    Eigen::MatrixXf degreeMatrix;
    degreeMatrix = Eigen::MatrixXf::Constant(validCloudSize,validCloudSize,0);
    Eigen::MatrixXf laplacianMatrix;
    // pcl::search::KdTree<PointXYZTP>::Ptr tree (new pcl::search::KdTree<PointXYZTP>);
    // tree->setInputCloud (cloud);
    KdTreeFLANN<PointXYZTP> tree;
    tree.setInputCloud(cloud_valid);
    //  PointXYZTP pointToSearch;
    //  pointToSearch.x = 0;
    //     pointToSearch.y = 0;
    //     pointToSearch.z = 0;
    //     pointToSearch.theta = 0;
    //     pointToSearch.phi = 0;
    //     std::vector<int> pointIdx;
    //     std::vector<float> pointDist;
    //     tree.radiusSearch(pointToSearch,searchRadius,pointIdx,pointDist,0);
    //  for (pcl::PointCloud<PointXYZTP>::iterator it = cloud->points.begin (); it != cloud->points.end (); ++it)
    for (size_t it=0;it<cloudIdx.size();++it)
    {
      std::vector<int> pointIdx;
      std::vector<float> pointDist;
      PointXYZTP pointToSearch;
      pointToSearch.x = cloud_valid->points[cloudIdx[it]].x;
      pointToSearch.y = cloud_valid->points[cloudIdx[it]].y;
      pointToSearch.z = cloud_valid->points[cloudIdx[it]].z;
      pointToSearch.theta = cloud_valid->points[cloudIdx[it]].theta;
      pointToSearch.phi = cloud_valid->points[cloudIdx[it]].phi;
      tree.radiusSearch(pointToSearch,searchRadius,pointIdx,pointDist,0);
      //    cout <<"point x:"<<pointToSearch.x <<"\t y:"<<pointToSearch.y<<"\t z:"<<pointToSearch.z<<"\t theta:"<<pointToSearch.theta<<"\t phi:"<<pointToSearch.phi<<"\t having neighbor points:"<<pointIdx.size()<<endl;

      if(pointIdx.size()==0)
      {
        cout <<"no neighbor "<<endl;
        continue;
      }
      // get the degree matrix D
      //    degreeMatrix(cloudIdx[it],cloudIdx[it]) = pointIdx.size()-1;
      int nCount = 0;
      for (size_t i=0;i<pointIdx.size();++i)
      {
        double dist = std::sqrt(pointDist[i]+pow((pointToSearch.theta-cloud_valid->points[pointIdx[i]].theta)*searchRadius/frontMaxAnlge,2)+pow((pointToSearch.phi-cloud_valid->points[pointIdx[i]].phi)*searchRadius/backMaxAnlge,2));
        //confAdjacencyMatrix(cloudIdx[it],pointIdx[i]) = 1;
        // confAdjacencyMatrix(pointIdx[i],cloudIdx[it]) = dist;
        //      cout <<"neighbor index:"<<pointIdx[i]<<"\t distance:"<<dist<<endl;
        //  cout <<"neighbot point x:"<<cloud->points[pointIdx[i]].x <<"\t y:"<<cloud->points[pointIdx[i]].y<<"\t z:"<<cloud->points[pointIdx[i]].z<<"\t theta:"<<cloud->points[pointIdx[i]].theta<<"\t phi:"<<cloud->points[pointIdx[i]].phi<<"\t having distance:"<<pointDist[i]<<endl;
        // get adjacency matrix A
        //      if(pointDist[i]>0)
        if( dist!=0 && dist < thresholdDist)
        {
          nCount++;
          confAdjacencyMatrix(cloudIdx[it],pointIdx[i]) = 1;
        }
      }
      //    cout << "valid neighbor count :"<<nCount<<endl;
      degreeMatrix(cloudIdx[it],cloudIdx[it]) = nCount;
      //   tree->radiusSearch(pointToSearch,searchRadius,pointIdx,pointDist,0);
    }
    // get laplacian matrix L = D - A
    laplacianMatrix = degreeMatrix - confAdjacencyMatrix;
//    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> es(laplacianMatrix);
    //  Eigen::EigenSolver<Eigen::MatrixXf> es(confAdjacencyMatrix);
    //  cout <<"adjacency matrix:"<<endl<< laplacianMatrix.diagonal()<<endl;
    // cout <<"adjacency eigen values:"<<endl<< es.eigenvalues() <<endl;
//    Eigen::VectorXf eigValues(es.eigenvalues());
//    int vectorSize = eigValues.rows();
//    Eigen::VectorXf  eigDiff(eigValues.tail(vectorSize-1)-eigValues.head(vectorSize-1));
//    double eigMean = eigDiff.mean();
//    cout <<"mean difference of eigen values :"<< eigMean <<endl;

    //      if (eigfile.is_open())
    //      {
//    eigfile.open(saveEigenMeanFile.str().c_str() ,ios::app);
//    eigfile << eigMean << '\n';
//    eigfile.close();
    //      }

    std::stringstream saveFile;
    saveFile <<"/home/mudassir/phd_ws/data/RSS2015_data/eigen_values/" << inputFile.c_str() <<dInx<< "_" << idx <<"_adj";
    std::ofstream savEigfile(saveFile.str().c_str());
    if (savEigfile.is_open())
    {
      savEigfile << confAdjacencyMatrix << "\n";
      savEigfile.close();
    }

    std::stringstream saveDeg;
    saveDeg <<"/home/mudassir/phd_ws/data/RSS2015_data/eigen_values/" << inputFile.c_str() <<dInx<< "_" << idx <<"_deg";
        std::ofstream savDegfile(saveDeg.str().c_str());
        if (savDegfile.is_open())
        {
          savDegfile << degreeMatrix.diagonal() << "\n";
          savDegfile.close();
        }
  }
  }
  return 0;
}
