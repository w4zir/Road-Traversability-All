#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;
using namespace pcl;

int main(int argc, char** argv) {
  std::string line;


  ifstream dataFile ("/home/mudassir/phd_ws/data/traversability/KITTI_dataset_raw/2011_09_26/2011_09_26_drive_0117_sync/oxts/data/0000000000.txt");

  if (dataFile.is_open())
  {
    std::getline (dataFile,line);
    std::istringstream iss(line);
    float lat;
    float lon;
    float alt;
    if (iss >> lat >> lon >> alt)
    {
      std::cout<< "lat:" << lat <<"\t long:" <<lon << "\t alt:" << alt << std::endl;
    }
    dataFile.close();
  }


  //    if (dataFile.is_open())
  //      {
  //        //cout<< "writing data to road_"<<fileIdx<<endl;
  //        while (!dataFile.eof())
  //        {
  //          getline (dataFile,line);
  //                size_t space1 = line.find(' ');
  //                size_t space2 = line.find(' ',space1+1);
  //                 string xStr = line.substr(0,space1);
  //                 string yStr = line.substr(space1,space2);
  //                 string zStr = line.substr(space2,line.size());
  //                 double lat = atof(xStr.c_str());
  //                 double lon = atof(yStr.c_str());
  //                 double alt = atof(zStr.c_str());
  //                 std::cout<< "lat:" << lat <<"\t long:" <<lon << "\t alt:" << alt << std::endl;
  //        }
  //        dataFile.close();
  //      }

  return 0;
}
