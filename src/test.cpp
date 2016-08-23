#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <queue>
#include <list>

float vehicle_width_ = 1.58;
float vehicle_length_ = 2.2;
float front_track_ = 1.3;
float wheelbase_ = 1.78;
float tire_width_ = 0.25;

const static float PI_ = 3.14159;
const static float MAX_PHI_ = 40;
const static float MAX_CLEARANCE_ = 5;

float road_max_x_ = 3;
float road_min_x_ = -3;
float road_max_y_ = 10;
float road_min_y_ = -10;

bool checkPointUnderVehicle (Eigen::Vector3f p, Eigen::MatrixXf vehicle_body_state)
{
    /*
     * A,B,C and D are corners of the vehicle state. let p be the point then the equation is (0 <= AP.AB <= AB.AB) && (0 <= AP.AD <= AD.AD)
     */
    Eigen::Vector3f a_point = vehicle_body_state.col(0);
    Eigen::Vector3f b_point = vehicle_body_state.col(1);
    Eigen::Vector3f d_point = vehicle_body_state.col(3);

    Eigen::Vector3f ap_vector = p - a_point;
    Eigen::Vector3f ab_vector = b_point - a_point;
    Eigen::Vector3f ad_vector = d_point - a_point;


    float ab_dot = ab_vector.dot(ab_vector);
    float apb_dot = ap_vector.dot(ab_vector);
    if (apb_dot < 0 || apb_dot > ab_dot)
    {
        return false;
    }

    float ad_dot = ad_vector.dot(ad_vector);
    float apd_dot = ap_vector.dot(ad_vector);
    if (apd_dot < 0 || apd_dot > ad_dot)
    {
        return false;
    }

    return true;
}

//#include "boost/tuple/tuple.hpp"
Eigen::MatrixXf getVehicleState (Eigen::Vector3f v_config)
{
    Eigen::MatrixXf vehicle_state_origin(3,8);
    vehicle_state_origin << -vehicle_width_/2,vehicle_width_/2,vehicle_width_/2,-vehicle_width_/2,-front_track_/2,front_track_/2, front_track_/2, -front_track_/2,
            -vehicle_length_/2+wheelbase_/2,-vehicle_length_/2+wheelbase_/2,vehicle_length_/2+wheelbase_/2,vehicle_length_/2+wheelbase_/2,0,0,wheelbase_,wheelbase_,
            0,0,0,0,0,0,0,0;

    /*
     *  Find the vehicle position using its configuration.
     */
    Eigen::Vector3f normal_xy_plane (0.0,0.0,1.0);
    Eigen::AngleAxis<float> transform((PI_*(v_config(2)-90))/(float)180,normal_xy_plane);
    Eigen::Matrix3f tranform_matrix;
    tranform_matrix = transform.matrix();
    Eigen::MatrixXf vehicle_state_rotated(3,8);
    vehicle_state_rotated = tranform_matrix*vehicle_state_origin;

    Eigen::MatrixXf position_offset(3,8);
    position_offset << v_config.head(2).replicate(1,8) , Eigen::MatrixXf::Zero(1,8);
    Eigen::MatrixXf vehicle_state(3,8);
    vehicle_state = vehicle_state_rotated + position_offset;
    return vehicle_state;
}


bool checkCircleCollisdeWithRectangle (Eigen::Vector3f circle, Eigen::MatrixXf vehicle_body_state)
{
    bool is_point_under_vehicle = checkPointUnderVehicle (circle,vehicle_body_state);
    //std::cout << "is_point_under_vehicle:\t" << is_point_under_vehicle << std::endl;
    if(is_point_under_vehicle)
    {
         std::cout << "obstacle center under vehicle body." << std::endl;
        return (true);
    }

    /*
     * Check whether a circle at point p and radius r collide with corners of a rectangular vehicle
     * A,B,C and D are corners of the vehicle state. The equation is (dist_ab <= r) for all corners
     */
    Eigen::Vector3f a_point = vehicle_body_state.col(0);
    Eigen::Vector3f b_point = vehicle_body_state.col(1);
    Eigen::Vector3f c_point = vehicle_body_state.col(2);
    Eigen::Vector3f d_point = vehicle_body_state.col(3);
    Eigen::Vector3f p = Eigen::Vector3f::Zero();
    p.head(2) = circle.head(2);

    float ap_dist = std::sqrt(std::pow(a_point(0)- circle(0),2) + std::pow(a_point(1)- circle(1),2));
    if(ap_dist <= circle(2))
    {
         std::cout << "obstacle collide with vehicle corner a.\t dist:" << ap_dist << std::endl;
        return (true);
    }
    float bp_dist = std::sqrt(std::pow(b_point(0)- circle(0),2) + std::pow(b_point(1)- circle(1),2));
    if(bp_dist <= circle(2))
    {
         std::cout << "obstacle collide with vehicle corner b.\t dist:" << bp_dist << std::endl;
        return (true);
    }
    float cp_dist = std::sqrt(std::pow(c_point(0)- circle(0),2) + std::pow(c_point(1)- circle(1),2));
    if(cp_dist <= circle(2))
    {
         std::cout << "obstacle collide with vehicle corner c.\t dist:" << cp_dist << std::endl;
        return (true);
    }
    float dp_dist = std::sqrt(std::pow(d_point(0)- circle(0),2) + std::pow(d_point(1)- circle(1),2));
    if(dp_dist <= circle(2))
    {
         std::cout << "obstacle collide with vehicle corner d.\t dist:" << dp_dist << std::endl;
        return (true);
    }

    /*
     * Check whether a circle at point p and radius r collide with sides of a rectangular vehicle
     * A,B,C and D are corners of the vehicle state. let p be the point then the equation is (0 <= AP.AB <= AB.AB) for all sides
     */

    // check for AB
    Eigen::Vector3f ap_vector = p - a_point;
    Eigen::Vector3f ab_vector = b_point - a_point;
    float ab_dot = ab_vector.dot(ab_vector);
    float pab_dot = ap_vector.dot(ab_vector);
    float p_ab_line_dist = std::sqrt(std::pow(ap_dist,2) - std::pow(std::sqrt(pab_dot),2));

    if (p_ab_line_dist < circle(2) &&  pab_dot >= 0 && pab_dot <= ab_dot)
    {
         std::cout << "obstacle collide with vehicle side ab." << std::endl;
        return (true);
    }

    // check for BC
    Eigen::Vector3f bp_vector = p - b_point;
    Eigen::Vector3f bc_vector = c_point - b_point;
    float bc_dot = bc_vector.dot(bc_vector);
    float pbc_dot = bp_vector.dot(bc_vector);
    float p_bc_line_dist = std::sqrt(std::pow(bp_dist,2) - std::pow(std::sqrt(pbc_dot),2));

    if (p_bc_line_dist < circle(2) &&  pbc_dot >= 0 && pbc_dot <= bc_dot)
    {
        std::cout << "obstacle collide with vehicle side bc." << std::endl;
        return (true);
    }

    // check for CD
    Eigen::Vector3f cp_vector = p - c_point;
    Eigen::Vector3f cd_vector = d_point - c_point;
    float cd_dot = cd_vector.dot(cd_vector);
    float pcd_dot = cp_vector.dot(cd_vector);
    float p_cd_line_dist = std::sqrt(std::pow(cp_dist,2) - std::pow(std::sqrt(pcd_dot),2));

    if (p_cd_line_dist < circle(2) &&  pcd_dot >= 0 && pcd_dot <= cd_dot)
    {
        std::cout << "obstacle collide with vehicle side cd." << std::endl;
        return (true);
    }

    // check for DA
    Eigen::Vector3f dp_vector = p - d_point;
    Eigen::Vector3f da_vector = a_point - d_point;
    float da_dot = da_vector.dot(da_vector);
    float pda_dot = dp_vector.dot(da_vector);
    float p_da_line_dist = std::sqrt(std::pow(dp_dist,2) - std::pow(std::sqrt(pda_dot),2));

    if (p_da_line_dist < circle(2) &&  pda_dot >= 0 && pda_dot <= da_dot)
    {
        std::cout << "obstacle collide with vehicle side da." << std::endl;
        return (true);
    }
    return false;
}


int collisionCheckerWithKnownObstacles (Eigen::MatrixXf vehicle_state, Eigen::MatrixXf obstacles_info_)
{
    /*
     *  Check if any of the tyres are outside road bounds.
     */
    float tyres_min_x = vehicle_state.row(0).rightCols(4).minCoeff();
    float tyres_max_x = vehicle_state.row(0).rightCols(4).maxCoeff();
    if(tyres_min_x < road_min_x_ || tyres_max_x > road_max_x_)
    {
         std::cout << "tyre not on road." << std::endl;
        return -1;
    }

    /*
     *  Check tyres for collision.
     */
    // std::cout << "obstacles_info_.rows() \t" << obstacles_info_.rows() << std::endl;
    for (int obs_idx = 0; obs_idx < obstacles_info_.rows(); obs_idx++)
    {
        for (int tyre_id=0; tyre_id < 4; tyre_id++)
        {
            float tyre_x = vehicle_state(0,4+tyre_id);
            float tyre_y = vehicle_state(1,4+tyre_id);
            float tyre_z = vehicle_state(2,4+tyre_id);
            float tire_dist_from_obs = std::sqrt(std::pow(obstacles_info_(obs_idx,0) - tyre_x,2) + std::pow(obstacles_info_(obs_idx,1) - tyre_y,2));
            if (tire_dist_from_obs <= (tire_width_/2 + obstacles_info_(obs_idx,2)))
            {
                 std::cout << "tyre collide with obstacle." << std::endl;
                return -1;
            }
        }
    }
    /*
     *  Check vehicle body for collision.
     */
    for (int obs_idx = 0; obs_idx < obstacles_info_.rows(); obs_idx++)
    {
        if (obstacles_info_(obs_idx,3) == -1)
        {
            continue;
        }
        //std::cout << "calling checkCircleCollisdeWithRectangle."<< std::endl;
        bool is_vehicle_body_collide = checkCircleCollisdeWithRectangle(obstacles_info_.row(obs_idx).head(3), vehicle_state.leftCols(4));
        //std::cout << "is_vehicle_body_collide:"<< is_vehicle_body_collide << std::endl;
        if (is_vehicle_body_collide)
        {
             std::cout << "vehicle body collide with obstacle." << std::endl;
            return -1;
        }

    }
    // std::cout << "end of func collisionCheckerWithKnownObstacles" << std::endl;
    return 1;
}

int main(int argc, char** argv) {

    /*
     * reading obstacle file
     */
     std::string pointcloudFolder = "/home/mudassir/phd_ws/traversability/pointclouds/clearance_roads_positive/";

    Eigen::MatrixXf obstacles_info_tmp = Eigen::MatrixXf::Constant(1000,4,-1000);
    Eigen::MatrixXf obstacles_info;
    std::stringstream readObstaclesInfoFile;
    readObstaclesInfoFile << pointcloudFolder.c_str() << "road_clear38_obs";
    std::cout <<"file name:\t" << readObstaclesInfoFile.str() << std::endl;
    std::ifstream obsFile (readObstaclesInfoFile.str().c_str());
    if (obsFile.is_open())
    {

        //cout<< "writing data to road_"<<fileIdx<<endl;
        int c_id = 0;
        while (!obsFile.eof())
        {
            std::string line;
            std::getline (obsFile,line);
            if(line.size()==0)
            {
                break;
            }
            std::size_t nospace1 = line.find_first_not_of(" ");
            std::size_t space1 = line.find(" ",nospace1);
            std::size_t nospace2 = line.find_first_not_of(" ",space1);
            std::size_t space2 = line.find(" ",nospace2);
            std::size_t nospace3 = line.find_first_not_of(" ",space2);
            std::size_t space3 = line.find(" ",nospace3);
            std::size_t nospace4 = line.find_first_not_of(" ",space3);
            std::size_t space4 = line.find(" ",nospace4);

            std::string xStr = line.substr(nospace1,space1-1);
            std::string yStr = line.substr(nospace2,space2-1);
            std::string rStr = line.substr(nospace3,space3-1);
            std::string sStr = line.substr(nospace4,space4-1);
            float xVal = std::atof(xStr.c_str());
            float yVal = std::atof(yStr.c_str());
            float rVal = std::atof(rStr.c_str());
            float sVal = std::atof(sStr.c_str());
           // std::cout <<c_id << ":\t line:	" << line <<"\t," << space1 <<"," << space2 <<"," << space3 << std::endl;

            obstacles_info_tmp.row(c_id) = Eigen::Vector4f(xVal,yVal,rVal,sVal);
            c_id++;
            //std::cout <<c_id << ":\t" << xVal <<"," << yVal <<"," << rVal <<"," << sVal << std::endl;

        }
        obsFile.close();
        obstacles_info = obstacles_info_tmp.topRows(c_id);
    }
    std::cout << obstacles_info.rows() <<" obstacles found.\n" << obstacles_info << std::endl;

    /*
     * Reading config files
     */
    std::string configFolder = "/home/mudassir/phd_ws/traversability/configs/clearance_roads_positive/rand_01_1/";

    Eigen::MatrixXf vehicle_configs_tmp = Eigen::MatrixXf::Constant(20000,4,-1000);
    Eigen::MatrixXf vehicle_configs;
    std::stringstream readConfigsFile;
    readConfigsFile << configFolder.c_str() << "road_clear38_conf";
    std::cout <<"file name:\t" << readConfigsFile.str() << std::endl;
    std::ifstream confFile (readConfigsFile.str().c_str());
    if (confFile.is_open())
    {

        //cout<< "writing data to road_"<<fileIdx<<endl;
        int c_id = 0;
        while (!confFile.eof())
        {
            std::string line;
            std::getline (confFile,line);
            if(line.size()==0)
            {
                break;
            }
            std::size_t nospace1 = line.find_first_not_of(" ");
            std::size_t space1 = line.find(" ",nospace1);
            std::size_t nospace2 = line.find_first_not_of(" ",space1);
            std::size_t space2 = line.find(" ",nospace2);
            std::size_t nospace3 = line.find_first_not_of(" ",space2);
            std::size_t space3 = line.find(" ",nospace3);
            std::size_t space_last = line.find_last_of(" ");
            //std::size_t nospace_last = line.find_first_not_of(" ",space_last);

            std::string xStr = line.substr(nospace1,space1-1);
            std::string yStr = line.substr(nospace2,space2-1);
            std::string rStr = line.substr(nospace3,space3-1);
            std::string sStr = line.substr(space_last-1,space_last-1);
            float xVal = std::atof(xStr.c_str());
            float yVal = std::atof(yStr.c_str());
            float rVal = std::atof(rStr.c_str());
            float sVal = std::atof(sStr.c_str());
            // std::cout <<c_id << ":\t line:	" << line <<"\t," << space1 <<"," << space2 <<"," << space3 << std::endl;

            vehicle_configs_tmp.row(c_id) = Eigen::Vector4f(xVal,yVal,rVal,sVal);
            c_id++;
           // std::cout <<c_id << ":\t" << xVal <<"," << yVal <<"," << rVal <<"," << sVal << std::endl;

        }
        confFile.close();
        vehicle_configs = vehicle_configs_tmp.topRows(c_id);
    }
    std::cout << vehicle_configs.rows() <<" configs found.\n" << std::endl;

    int same_flag = 0;
    for (int cidx = 1; cidx < 100; cidx++) //vehicle_configs.rows()
    {
        Eigen::MatrixXf vehicle_state = getVehicleState(vehicle_configs.row(cidx).head(3));
       // std::cout << "vehicle state: " << vehicle_state << std::endl;
        int safety_flag = collisionCheckerWithKnownObstacles(vehicle_state, obstacles_info);
        std::cout <<cidx << "\t original flag:" << vehicle_configs(cidx,3) << "\t computed flag:" << safety_flag << "\t" << vehicle_configs.row(cidx).head(3) << std::endl;
        if (safety_flag == vehicle_configs(cidx,3))
        {
            same_flag++;
        }
    }
    std::cout << "same flag count:" << same_flag << std::endl;
    return 0;
}
