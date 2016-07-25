/*
 * get_sensors_data.cpp
 *
 ************************************************************************************************
lat:   latitude of the oxts-unit (deg)
lon:   longitude of the oxts-unit (deg)
alt:   altitude of the oxts-unit (m)
roll:  roll angle (rad),    0 = level, positive = left side up,      range: -pi   .. +pi
pitch: pitch angle (rad),   0 = level, positive = front down,        range: -pi/2 .. +pi/2
yaw:   heading (rad),       0 = east,  positive = counter clockwise, range: -pi   .. +pi
vn:    velocity towards north (m/s)
ve:    velocity towards east (m/s)
vf:    forward velocity, i.e. parallel to earth-surface (m/s)
vl:    leftward velocity, i.e. parallel to earth-surface (m/s)
vu:    upward velocity, i.e. perpendicular to earth-surface (m/s)
ax:    acceleration in x, i.e. in direction of vehicle front (m/s^2)
ay:    acceleration in y, i.e. in direction of vehicle left (m/s^2)
ay:    acceleration in z, i.e. in direction of vehicle top (m/s^2)
af:    forward acceleration (m/s^2)
al:    leftward acceleration (m/s^2)
au:    upward acceleration (m/s^2)
wx:    angular rate around x (rad/s)
wy:    angular rate around y (rad/s)
wz:    angular rate around z (rad/s)
wf:    angular rate around forward axis (rad/s)
wl:    angular rate around leftward axis (rad/s)
wu:    angular rate around upward axis (rad/s)
pos_accuracy:  velocity accuracy (north/east in m)
vel_accuracy:  velocity accuracy (north/east in m/s)
navstat:       navigation status (see navstat_to_string)
numsats:       number of satellites tracked by primary GPS receiver
posmode:       position mode of primary GPS receiver (see gps_mode_to_string)
velmode:       velocity mode of primary GPS receiver (see gps_mode_to_string)
orimode:       orientation mode of primary GPS receiver (see gps_mode_to_string)
 ************************************************************************************************
 *
 *  Created on: Nov 25, 2015
 *      Author: mudassir
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


int main(int argc, char** argv)
{
  double const PI = 3.14159265;
  std::vector <std::string> file_names;

  std::ifstream dataFile ("/home/mudassir/phd_ws/data/traversability/KITTI_dataset_raw/2011_09_26/2011_09_26_drive_0117_sync/oxts/data/0000000000.txt");
  std::string dataFolder = "/home/mudassir/phd_ws/data/traversability/KITTI_dataset_raw/2011_09_26/2011_09_26_drive_0117_sync/oxts/data/";

  int file_no = 0;
  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir (dataFolder.c_str())) != NULL)
  {
    while ((ent = readdir (dir)) != NULL)
    {
      std::string fName = ent->d_name;
      file_names.push_back(fName);
    }
    std::sort( file_names.begin(), file_names.end() );

    for (int i=0;i<file_names.size();i+=5)
    {
      std::stringstream file;
      file << dataFolder.c_str() <<file_names[i] ;
      std::ifstream file_in(file.str().c_str());


      //      std::cout << file_names[i] << std::endl;

      std::string line;
      if (file_in.is_open())
      {
        std::getline (file_in,line);
        std::istringstream iss(line);
        double lat;
        double lon;
        double alt;
        double roll;
        double pitch;
        double yaw;
        double vn;
        double ve;
        double vf;
        double vl;
        double vu;
        double ax;
        double ay;
        double af;
        double al;
        double au;
        double wx;
        double wy;
        double wz;
        double wf;
        double wl;
        double wu;
        if (iss >> lat >> lon >> alt >> roll >> pitch >> yaw)
        {
          std::cout<< "file:" << i << "\t lat:" << lat <<"\t long:" <<lon << "\t alt:" << alt << "\t yaw:" << yaw*180/PI << std::endl;
        }
        file_in.close();
      }
    }
    closedir (dir);
  } else {
    perror ("could not open directory");
    return EXIT_FAILURE;
  }

  return 0;
}


