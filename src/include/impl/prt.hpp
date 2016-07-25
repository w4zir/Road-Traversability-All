/*
 * height_estimate.hpp
 *
 *  Created on: Dec 1, 2015
 *      Author: mudassir
 */

#ifndef PRT_HPP_
#define PRT_HPP_

#include <prt.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::PRT<PointT>::PRT () :
configs_limit_ (5000),
config_x_count_ (25),
config_y_count_ (81),
config_theta_count_ (7),
start_configs_ (20),
config_counter_ (0),
min_theta_ (45),
max_theta_ (135),
vehicle_velocity_ (0.5),
vehicle_steering_rate_ (12),
vehicle_width_ (1.58), //2
vehicle_length_ (2.2), //3
vehicle_heigt_ (1.9), //1.5
wheelbase_ (1.78), //2.2
front_track_ (1.3), //1.4
ground_clearance_ (0.3), //0.4
tire_width_ (0.25), //0.2
negative_obs_threshold_ (0.2), //0.2
config_neighbour_radius_ (2.0), //0.3
prm_step_size_ (0.2), 
sub_config_count_(2), //2
vehicle_safety_dist_ (0),
goal_tolerance_ (1.0),
goal_bias_ (0.05),
vehicle_configs_ (),
config_neighbours_ (),
config_collision_status_ (),
config_clearance_ (),
vehicle_state_ (),
dem_search_ (),
config_search_ (),
cloud_config_ (),
is_import_configs_ (false),
is_random_configs_ (false),
prt_graph_ ()
{
	//search_ =  typename pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::PRT<PointT>::~PRT ()
{
	if (dem_search_ != 0)
		dem_search_.reset ();

	if (config_search_ != 0)
		config_search_.reset ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::PRT<PointT>::getRandomConfigsFlag () const
{
	return is_random_configs_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRT<PointT>::setRandomConfigsFlag (bool is_random_configs)
{
	is_random_configs_ = is_random_configs;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::PRT<PointT>::getImportConfigsFlag () const
{
	return is_import_configs_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRT<PointT>::setImportConfigsFlag (bool is_import_configs)
{
	is_import_configs_ = is_import_configs;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRT<PointT>::getPRTAdjacencyList (std::vector< std::pair<int,int> >& prt_graph)
{
	prt_graph = prt_graph_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRT<PointT>::setGoalBiasProbability (float goal_bias)
{
	goal_bias_ = goal_bias;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::PRT<PointT>::getGoalBiasProbability () const
{
	return (goal_bias_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::PRT<PointT>::getConfigCounter () const
{
	return (config_counter_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> std::vector<int> 
pcl::PRT<PointT>::getConfigurationsValidityStatus ()
{
	return config_collision_status_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> std::vector<float> 
pcl::PRT<PointT>::getConfigurationsClearance ()
{
	return config_clearance_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> Eigen::MatrixXf
pcl::PRT<PointT>::getConfigurations ()
{
	return vehicle_configs_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRT<PointT>::setConfigurations (Eigen::MatrixXf vehicle_configs)
{
	vehicle_configs_ = vehicle_configs;
	vehicle_configs_.col(3) = Eigen::MatrixXf::Constant(vehicle_configs.rows(),1,0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> Eigen::Vector3f
pcl::PRT<PointT>::generateRandomConfiguration ()
{
	float rand_flip =  rand () / (float)(RAND_MAX + 1.0f);
	float y_val;
	if(rand_flip < goal_bias_)
	{
		y_val = rand () / (float)(RAND_MAX + 1.0f) + (road_max_y_ - goal_tolerance_);
	} else
	{
		y_val = (road_max_y_ - road_min_y_) * rand () / (float)(RAND_MAX + 1.0f) + road_min_y_;
	}
	float x_val = (road_max_x_ - road_min_x_) * rand () / (float)(RAND_MAX + 1.0f) + road_min_x_;
	float t_val = (max_theta_ - min_theta_) * rand () / (float)(RAND_MAX + 1.0f) + min_theta_;

	return Eigen::Vector3f(x_val,y_val,t_val);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::PRT<PointT>::findQrandNearestNeighbor (Eigen::Vector3f qrand)
{
	float min_config_dist = 10000;
	int config_idx  = -1;
	for (int c_idx = 0; c_idx < config_counter_; c_idx++ )
	{
		float config_distance = config_dist(qrand,vehicle_configs_.row(c_idx)); 
		if(config_distance < min_config_dist)
		{
			min_config_dist = config_distance;
			config_idx = c_idx;
		}
	}
	return (config_idx);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::PRT<PointT>::config_dist (Eigen::Vector3f qrand, Eigen::Vector4f qnear)
{	
	float theta_diff = PI_*(qrand(2) - qnear(2))/(float)180;

	if (theta_diff > PI_)
	{
		theta_diff -= 2*PI_;
	}
	else if (theta_diff < -PI_)
	{
		theta_diff += 2*PI_;
	}

	float dist = (qrand(0)-qnear(0))*(qrand(0)-qnear(0))+ (qrand(1)-qnear(1))*(qrand(1)-qnear(1))+ wheelbase_*wheelbase_*theta_diff*theta_diff;
	dist = std::pow(dist,0.5);

	return(dist);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> Eigen::Vector4f
pcl::PRT<PointT>::findOptimalConfig (Eigen::Vector3f qrand, Eigen::Vector4f qnear)
{
	Eigen::Vector4f q_new;
	float step_dist = vehicle_velocity_;

	float x_new;
	float y_new;
	float theta_new;
	float phi_new;
	float min_config_dist = 10000;

	for (int direction = -1; direction <= 1; direction++ )
	{
		float phi = qnear(3) + direction*vehicle_steering_rate_;
		float beta = (step_dist*tan(phi*PI_/(float)180)/wheelbase_);
		//		std::cout << "beta: " << beta << std::endl;

		if(std::abs(phi) > MAX_PHI_)
		{
			continue;
		}else if(std::abs(beta) < 0.001)
		{
			x_new = qnear(0)+step_dist*cos(PI_*qnear(2)/(float)180);
			y_new = qnear(1)+step_dist*sin(PI_*qnear(2)/(float)180);
			theta_new = qnear(2);
			phi_new = qnear(3);
		}else
		{
			//			float turning_radius = wheelbase_ / (float)std::tan(PI_*beta/(float)180);
			float turning_radius = step_dist/(beta);
			float xc = qnear(0) - sin(PI_*qnear(2)/(float)180)*turning_radius;
			float yc = qnear(1) + cos(PI_*qnear(2)/(float)180)*turning_radius;
			x_new = xc + sin((PI_*qnear(2)/(float)180)+beta)*turning_radius;
			y_new = yc - cos((PI_*qnear(2)/(float)180)+beta)*turning_radius;
			theta_new = (qnear(2)+beta*180/PI_);//((qnear(2)+beta)%(float)(2*PI_));
			phi_new = phi;
		}
		Eigen::Vector4f qnew(x_new,y_new,theta_new,phi_new);
		float config_distance = config_dist(qrand,qnew); 
		//		std::cout << "qnew: " << qnew.transpose() << "\t dist:" << config_distance << std::endl;
		if(config_distance < min_config_dist)
		{
			min_config_dist = config_distance;
			q_new = qnew;
		}
	}
	return (q_new);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::PRT<PointT>::checkGoalReached (Eigen::Vector4f q_new)
{
	if (q_new(1) > std::abs(road_max_y_ - goal_tolerance_))
	{
		return true;
	} else
	{
		return false;
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::PRT<PointT>::checkConfigSafety (Eigen::Vector4f config)
{
	Eigen::MatrixXf vehicle_state = getVehicleStateWithClearance(config.head(3));
	//		std::cout << "vehicle state: " << vehicle_state << std::endl;
	//		float state_min_x = vehicle_state.row(1).leftCols(4).minCoeff();
	int config_safe = collisionAndSafetyChecker(vehicle_state);
	return (config_safe);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRT<PointT>::addStartConfigurations ()
{
	config_counter_ = 0;
	int start_attempts = 0;
	while(config_counter_ < start_configs_ && start_attempts < 5*start_configs_)
	{
		float s_x = (road_max_x_ - road_min_x_ - 2*vehicle_safety_dist_ - vehicle_width_) * rand () / (RAND_MAX + 1.0f) + road_min_x_ + vehicle_safety_dist_ + vehicle_width_/2;
		float s_y = (0.5) * rand () / (RAND_MAX + 1.0f) + road_min_y_;
		float s_theta = (max_theta_ - min_theta_) * rand () / (RAND_MAX + 1.0f) + min_theta_;
		float s_phi = 0;
		Eigen::Vector4f s_config(s_x,s_y,s_theta,s_phi);

		//std::cout << "checking start safety." << std::endl;
		int is_q_safe = checkConfigSafety(s_config);
		//std::cout << "start is safe." << std::endl;
		if(is_q_safe == 1)
		{
			vehicle_configs_.row(config_counter_) = s_config;
			config_counter_++;
		}
		start_attempts++;
	}
	if(start_attempts == 5*start_configs_)
	{
		std::cout << "Could not find a safe start configuration. Exiting... " << std::endl;
		exit(0);
	}else 
	{
		std::cout << "Start configurations added successfully." << std::endl;
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRT<PointT>::getConfigCloud ()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr prm_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	for (int i=0; i < vehicle_configs_.rows(); i++)
	{
		pcl::PointXYZ point;
		point.x = vehicle_configs_.row(i)[0];
		point.y = vehicle_configs_.row(i)[1];
		point.z = 0;
		prm_cloud->push_back(point);
	}
	cloud_config_ = prm_cloud;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRT<PointT>::findConfigNeighbours ()
{
	int config_count = static_cast<int> (cloud_config_->points.size ());
	std::vector<int> neighbours;
	std::vector<float> distances;

	//		int n_zero = 0;
	//	  std::cout << "config_count:" << config_count << std::endl;
	config_neighbours_.resize (config_count, neighbours);
	for (int i_point = 0; i_point < config_count; i_point++)
	{
		neighbours.clear ();
		pcl::PointXYZ point;
		point.x = cloud_config_->points[i_point].x;
		point.y = cloud_config_->points[i_point].y;
		point.z = cloud_config_->points[i_point].z;
		//		config_search_ ->nearestKSearch (point, 9*config_theta_count_, neighbours, distances);
		config_search_ ->radiusSearch (point, config_neighbour_radius_, neighbours, distances);
		//				if(neighbours.size()<=1)
		//					n_zero++;
		config_neighbours_[i_point].swap (neighbours);
	}
	//	std::cout <<"zero neighbor count:" << n_zero << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> Eigen::MatrixXf
pcl::PRT<PointT>::getNeighborSubConfigs(Eigen::Vector4f c_config, Eigen::Vector4f n_config, float neighbor_ratio)
{
	/*
	 * loop untill reach neighbor
	 */
//	std::cout << "c_config:" << c_config.transpose() << "\t n_config:" << n_config.transpose() << "\t neighbor_ratio" << neighbor_ratio << std::endl;
	Eigen::MatrixXf sub_configs_temp = Eigen::MatrixXf::Constant(20,3,-1000);
	float phi = -MAX_PHI_ + (2*MAX_PHI_)*neighbor_ratio;
	float radius_n = wheelbase_/tan(phi*PI_/(float)180);
	bool neighbor_reached = false;
	Eigen::Vector3f config = c_config.head(3);
	int index = 0;
	for(;;)
	{
		float beta = 180*((prm_step_size_*tan(phi*PI_/(float)180))/wheelbase_)/PI_;
		float xn = -1;
		float yn = -1;
		if (std::abs(beta) < 0.001)
		{
			xn = config(0) + prm_step_size_*(cos(PI_*config(2)/180));
			yn = config(1) + prm_step_size_*(sin(PI_*config(2)/180));
		} else
		{
			xn = config(0) + (sin(PI_*(config(2)+beta)/180) - sin(PI_*config(2)/180))*radius_n;
			yn = config(1) + (cos(PI_*config(2)/180) - cos(PI_*(config(2)+beta)/180))*radius_n;
		}
		float tn = config(2) + beta;
		config = Eigen::Vector3f(xn,yn,tn);
		float sub_2_n_dist = std::sqrt(std::pow(config(0)- n_config(0),2) + std::pow(config(1)- n_config(1),2));
		sub_configs_temp.row(index) = config;	
//		std::cout << "sub config:" << config.transpose() << "\t sub config counter:" << index << "\t sub_2_n_dist:" << sub_2_n_dist << "\t rows:" << sub_configs_temp.rows()<< std::endl;
		index++;
		if (sub_2_n_dist < prm_step_size_)
		{
//			std::cout << "sub_configs_temp.topRows(index):" << sub_configs_temp.topRows(index).transpose() << std::endl;
			Eigen::MatrixXf sub_configs = sub_configs_temp.topRows(index);
			return sub_configs;
		}
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> Eigen::MatrixXf
pcl::PRT<PointT>::getConfigCircluarCenters (Eigen::Vector4f config) 
{
	Eigen::MatrixXf config_circles(2,3);

	int idx = 0;
	for (int direction = -1; direction <= 1; direction+=2 )
	{
		float phi = config(3) + direction*MAX_PHI_;

		float turning_radius = wheelbase_/tan(phi*PI_/(float)180);
		float xc = config(0) - sin(PI_*config(2)/(float)180)*turning_radius;
		float yc = config(1) + cos(PI_*config(2)/(float)180)*turning_radius;

		config_circles.row(idx) = Eigen::Vector3f(xc,yc,turning_radius);
		idx++;				
	}
	return config_circles;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//template <typename PointT> void
//pcl::PRT<PointT>::generatePRTGraph ()
//{
//	std::vector< std::pair<int,int> > prt_graph;
//	//float phi_step = vehicle_steering_rate_ / (float)(sub_config_count_ + 1);
//
//	bool is_goal = false;	
//	int counter = config_counter_;
//	std::cout << "config count at start of generatePRTGraph ():"<< config_counter_ << std::endl;
//	while (!is_goal && counter < configs_limit_)
//	{
//		Eigen::Vector3f q_rand = generateRandomConfiguration();
//		int q_near_idx = findQrandNearestNeighbor(q_rand);
//		Eigen::Vector4f q_new = findOptimalConfig(q_rand,vehicle_configs_.row(q_near_idx));
////		std::cout <<"Optimal config:"<< q_new.transpose() << std::endl;
//
//		int is_q_safe = checkConfigSafety(q_new);
//		if(is_q_safe == 1)
//		{			
//			vehicle_configs_.row(config_counter_) = q_new;
//			prt_graph.push_back(std::make_pair(q_near_idx,config_counter_));			
//			is_goal = checkGoalReached(q_new);
//			config_counter_++;
//		}
//		counter++;
//		//std::cout << "config counter:" << counter << "\t safe counter:" << config_counter_ <<"\t q:"<<q_new.transpose()<< std::endl;
//	}
//	std::cout << "config counter:" << counter << "\t safe counter:" << config_counter_ << std::endl;
//	prt_graph_ = prt_graph;
//	std::cout << "PRT graph generated." << std::endl;
//	if(is_goal)
//	{
//		std::cout << "Goal reached successfully !!!" << std::endl;
//	}else
//	{
//		std::cout << "Could not reach goal :(" << std::endl;
//	}
//}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRT<PointT>::validateConfigurations ()
{
	std::cout << "vehicle_configs_ rows: " << vehicle_configs_.rows() << "\t cols" << vehicle_configs_.cols() << std::endl;
	int config_count = vehicle_configs_.rows(); //config_x_count_ * config_y_count_ * config_theta_count_;
	config_collision_status_.resize(config_count,-1);
	config_clearance_.resize(config_count,-1);
	int valid_count=0;
	int invalid_count=0;
	int counter=0;
	for (int c_idx=0; c_idx < config_count; c_idx++)
	{
		//std::cout << "vehicle config:\t" << c_idx << std::endl;
		Eigen::MatrixXf vehicle_state = getVehicleState(vehicle_configs_.row(c_idx).head(3));
		//		Eigen::MatrixXf vehicle_state = getVehicleStateWithClearance(vehicle_configs_.row(c_idx).head(3));
		//	std::cout << "vehicle config: " << vehicle_configs_.row(c_idx).head(3) << std::endl;
		//std::cout << "vehicle state: " << vehicle_state << std::endl;
		//		float state_min_x = vehicle_state.row(1).leftCols(4).minCoeff();
		int safety_flag = collisionAndSafetyChecker(vehicle_state);
		if(safety_flag == 1)
		{
			config_collision_status_[c_idx] = 1;
			float config_clearance_value = 0;//computeConfigClearance(vehicle_configs_.row(c_idx).head(3), vehicle_state);
			config_clearance_ [c_idx] = config_clearance_value;
			//std::cout << "config_clearance : " << config_clearance_ [c_idx] << std::endl;
			valid_count++;
		} else
		{
			config_collision_status_[c_idx] = 0;
			config_clearance_ [c_idx] = 0;
			invalid_count++;
		}
		counter++;
	}

	//	default_dem_cells_ = tmp_dem_cells.leftCols(cell_counter);
	std::cout <<"valid:" << valid_count << "\t invalid:" << invalid_count <<"\t counter:" << counter << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::PRT<PointT>::findConfigsConnectivity (Eigen::Vector4f c_config, Eigen::Vector4f n_config)
{
	// neighbors ahead of the config
	if(c_config(1) >= n_config(1))
	{
		//		std::cout <<"neighbor config has lower y value."<< std::endl;
		return -1;
	}

	// check that there is a continuous differentiable and fixed curvature arc to neighbor	
	float n_dist = std::sqrt(std::pow(c_config(0)- n_config(0),2) + std::pow(c_config(1)- n_config(1),2));
	if (n_dist < (config_neighbour_radius_ - 0.5))
	{
		return -1;
	}

	/*
	 * Find pose while taking right turn with maximum turning angle
	 */
	float alpha_min = -40;
	float radius_n = wheelbase_/tan(alpha_min*PI_/(float)180);
	float beta = 180*(n_dist/radius_n)/PI_;
	float xn = c_config(0) + (sin(PI_*(c_config(2)+beta)/180) - sin(PI_*c_config(2)/180))*radius_n;
	float yn = c_config(1) + (cos(PI_*c_config(2)/180) - cos(PI_*(c_config(2)+beta)/180))*radius_n;
	float tn =  c_config(2) + beta;
	Eigen::Vector3f p_min (xn,yn,tn);

	/*
	 * Find pose while taking left turn with maximum turning angle
	 */
	float alpha_max = 40;
	radius_n = wheelbase_/tan(alpha_max*PI_/(float)180);
	beta = 180*(n_dist/radius_n)/PI_;
	xn = c_config(0) + (sin(PI_*(c_config(2)+beta)/180) - sin(PI_*c_config(2)/180))*radius_n;
	yn = c_config(1) + (cos(PI_*c_config(2)/180) - cos(PI_*(c_config(2)+beta)/180))*radius_n;
	tn =  c_config(2) + beta;
	Eigen::Vector3f p_max (xn,yn,tn);

	/*
	 * if neighbor orientation is outside bounds of left and right turn then its not reachable
	 */
	if(n_config(2) < p_min(2) || n_config(2) > p_max(2))
	{
		//		std::cout << "invalid due to out of reachable zone.\t" << p_min << p_max<< std::endl;
		return -1;
	}

	/*
	 * If distance of neighbor to p_min is less than zero or greater than the distance the distance 
	 * between p_min and p_max then its not reachable
	 */
	Eigen::Vector2f v1 = p_min.head(2) - n_config.head(2);
	Eigen::Vector2f v2 = p_min.head(2) - p_max.head(2);
	float d1 = v1.dot(v2)/v2.norm();
	float dmin_max = std::sqrt(std::pow(p_min(0)- p_max(0),2) + std::pow(p_min(1)- p_max(1),2)); 

	if(d1 < 0 || d1 > dmin_max)
	{
		//		std::cout << "invalid due to distance" << std::endl;
		return -1;
	}

	/*
	 *  If ratio of distance of neighbor from p_min to distance between p_min and p_max is equal to 
	 *  ratio of difference of angle of neighbor to p_min and difference between angle of p_min and p_max
	 *  with some margin, then neoighbor is reachable.
	 */
	float ratio1 = d1/dmin_max;
	float ratio2 = (p_min(2)-n_config(2))/(p_min(2)-p_max(2));

	if(std::abs(ratio1-ratio2) > 0.05)
	{
		//		std::cout << "invalid due to ratio" << std::endl;
		return -1;
	}

	return (ratio1+ratio2)/2;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::PRT<PointT>::findConfigsConnectivity (Eigen::Vector4f c_config, Eigen::Vector4f n_config, Eigen::MatrixXf config_circles)
{
	float theta_resolution = (max_theta_ - min_theta_)/(float)(config_theta_count_-1); 
	// neighbors ahead of the config
	if(c_config(1) >= n_config(1))
	{
		//		std::cout <<"neighbor config has lower y value."<< std::endl;
		return false;
	}

	// check whether its ahead of config and the heading difference is not too much
	float neighbor_to_config_angle = fmod((atan2((n_config(1)-c_config(1)),(n_config(0)-c_config(0)))*180/(float)PI_),360);
	//std::cout <<"neighbor_to_config_angle:" << neighbor_to_config_angle << " \t c_config(2):" << c_config(2) << std::endl;
	if(std::abs(neighbor_to_config_angle-c_config(2)) > (theta_resolution+5) || std::abs(neighbor_to_config_angle-n_config(2)) > (theta_resolution+5) )
	{
		return false;
	}


	// neighbors ahead of the config
	if(std::abs(c_config(2) - n_config(2)) > (theta_resolution+5))
	{
		//		std::cout <<"neighbor config has lower y value."<< std::endl;
		return false;
	}

	for (int i=0; i < config_circles.rows(); i++)
	{
		float n_center_dist = std::sqrt(std::pow(config_circles(i,0)- n_config(0),2) + std::pow(config_circles(i,1)- n_config(1),2));
		if(n_center_dist <= std::abs(config_circles(i,2)))
		{
			//			std::cout <<"neighbor config lies with circle of config. n_center_dist"<< n_center_dist << "\t config_circles(i,2):" << config_circles(i,2) << std::endl;
			return false;
		}
	}

	// Get neighbor config circular info
	//	Eigen::MatrixXf n_config_circles = getConfigCircluarCenters(n_config);


	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRT<PointT>::generatePRTGraph ()
{
	std::vector< std::pair<int,int> > prt_graph;

	for (int c_idx=0; c_idx < cloud_config_->points.size(); c_idx++)
		//for (int c_idx=0; c_idx <  config_count; c_idx++)
	{
		//std::cout <<"finding connectivity of node:" << c_idx << " with collision status:" << config_collision_status_[c_idx] << std::endl;

		if(config_collision_status_[c_idx] != 1)
		{
			continue;
		} else
		{
			/*
			 * Get neighbors configuration and see if they are reachable from current configuration.
			 */
			//			std::vector<int> neighbor_idx = config_neighbours_(c_idx);
			int neighbor_count = config_neighbours_[c_idx].size();
			//			std::cout <<"neighboring configuration count: \t"<< neighbor_count << std::endl;
			if(neighbor_count<=0)
			{
				continue;
			}
			
			// current config
			Eigen::Vector4f c_config(vehicle_configs_.row(c_idx));
			// Get config circular info
			//Eigen::MatrixXf config_circles = getConfigCircluarCenters(c_config);

			//			std::cout <<"config id:"<< c_idx << "\t coord:" << c_config.head(3).transpose() << std::endl;
			//			std::cout <<"config circle info:" << config_circles << std::endl;

			for (int n_idx=0; n_idx < neighbor_count; n_idx++)
			{
				int neighbor_indx = config_neighbours_[c_idx][n_idx];
				//				if(neighbor_idx[n_idx] > config_count)
				//				{
				//					continue;
				//				}
				// get neighbor config
				Eigen::Vector4f n_config(vehicle_configs_.row(neighbor_indx));
				//				std::cout <<"neighbor id:"<< neighbor_indx << "\t coord:" << n_config.head(3).transpose() << std::endl;

				if(config_collision_status_[neighbor_indx] != 1 )
				{
					//					std::cout <<"neighbor config is invalid."<< std::endl;
					continue;
				} else 
				{
					/*
					 * if neighbor is valid then sub-sample and find a valid path
					 */
					//	bool is_neighbor_reachable = findConfigsConnectivity(c_config,n_config,config_circles);
					float neighbor_ratio = findConfigsConnectivity(c_config,n_config);
					if(neighbor_ratio != -1)
					{
						Eigen::MatrixXf n_sub_configs = getNeighborSubConfigs(c_config,n_config,neighbor_ratio);
//						std::cout << "call return with n_sub_configs:" << n_sub_configs.transpose() << std::endl;
						//						Eigen::Vector4f config_diff = n_config - c_config;
						//						float n_dist = std::sqrt(std::pow(config_diff(0),2) + std::pow(config_diff(1),2));
						//						int sub_config_count = std::floor(n_dist/tire_width_);
						//						float xInc = (config_diff[0])/(sub_config_count + 1);
						//						float yInc = (config_diff[1])/(sub_config_count + 1);
						//						float tInc = (config_diff[2])/(sub_config_count + 1);

						//						for (int sc_idx=1; sc_idx <= sub_config_count; sc_idx++)
						for (int s_idx=0; s_idx < n_sub_configs.rows(); s_idx++)
						{
							Eigen::Vector3f sub_config = n_sub_configs.row(s_idx);
							//							Eigen::Vector4f sub_config(c_config(0) + xInc, c_config(1) + yInc, c_config(2) + tInc, 0);
							//							if(sub_config_count > 0)
							//							{
							//								std::cout << "c_config: " << c_config << "n_config: " << n_config << "sub_config_count: " << sub_config_count << "sub_config: " << sub_config << std::endl;
							//							}
							Eigen::MatrixXf vehicle_state = getVehicleState(sub_config.head(3));
							//		std::cout << "vehicle state: " << vehicle_state << std::endl;
							//		float state_min_x = vehicle_state.row(1).leftCols(4).minCoeff();
							int sub_config_valid_flag = collisionAndSafetyChecker(vehicle_state);
							//		std::cout << "sub-config:" << sub_config.transpose() << "\t validity:" << config_valid_flag << std::endl;
							if(sub_config_valid_flag == 1 && s_idx == (n_sub_configs.rows()-1)) 
							{
								prt_graph.push_back(std::make_pair(c_idx,neighbor_indx));
							}
						}
					}

				}
			}
		}
	}
	prt_graph_ = prt_graph;
	std::cout << " Graph generated with edges:\t" << prt_graph.size() << std::endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//template <typename PointT> void
//pcl::PRT<PointT>::generatePRTGraph ()
//{
//	std::vector< std::pair<int,int> > prt_graph;
//
//	for (int c_idx=0; c_idx < cloud_config_->points.size(); c_idx++)
//		//for (int c_idx=0; c_idx <  config_count; c_idx++)
//	{
//
//		int neighbor_count = config_neighbours_[c_idx].size();
//		//			std::cout <<"neighboring configuration count: \t"<< neighbor_count << std::endl;
//		if(neighbor_count<=0)
//		{
//			continue;
//		}
//
//		Eigen::Vector4f c_config(vehicle_configs_.row(c_idx));
//		// Get config circular info
//		Eigen::MatrixXf config_circles = getConfigCircluarCenters(c_config);
//
//		//			std::cout <<"config id:"<< c_idx << "\t coord:" << c_config.head(3).transpose() << std::endl;
//		//			std::cout <<"config circle info:" << config_circles << std::endl;
//
//		for (int n_idx=0; n_idx < neighbor_count; n_idx++)
//		{
//			int neighbor_indx = config_neighbours_[c_idx][n_idx];
//			//				if(neighbor_idx[n_idx] > config_count)
//			//				{
//			//					continue;
//			//				}
//			Eigen::Vector4f n_config(vehicle_configs_.row(neighbor_indx));
//			//				std::cout <<"neighbor id:"<< neighbor_indx << "\t coord:" << n_config.head(3).transpose() << std::endl;
//
//			/*
//			 * if neighbor is valid then sub-sample and find a valid path
//			 */
//			bool is_neighbor_reachable = findConfigsConnectivity(c_config,n_config,config_circles);
//			if(is_neighbor_reachable)
//			{
//				prt_graph.push_back(std::make_pair(c_idx,neighbor_indx));
//			}
//
//		}
//	}
//prt_graph_ = prt_graph;
//std::cout << " Graph generated with edges:\t" << prt_graph.size() << std::endl;
//}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRT<PointT>::generateConfigurations ()
{					
	int config_count = config_x_count_ * config_y_count_ * config_theta_count_;
	//	std::cout<<"x:"<< config_x_count_ <<"\t y:"<< config_y_count_ <<"\t t:"<< config_theta_count_ <<"\t total:"<< config_count<< std::endl;
	vehicle_configs_ = Eigen::MatrixXf::Constant(config_count,4,-1000);

	float config_min_x = road_min_x_;
	float config_max_x = road_max_x_;
	//	float config_min_x = road_min_x_ + tire_width_/2 + front_track_/2;
	//		float config_max_x = road_max_x_ - tire_width_/2 - front_track_/2;

	float x_resolution = (config_max_x - config_min_x)/(double)(config_x_count_ - 1);
	float y_resolution = (road_max_y_ - road_min_y_)/(double)(config_y_count_ - 1);
	float theta_resolution = (max_theta_ - min_theta_)/(double)(config_theta_count_ - 1);
	std::cout<<"x_resolution:"<< x_resolution <<"\t y_resolution:"<< y_resolution <<"\t theta_resolution:"<< theta_resolution << std::endl;

	for (int y_idx=0; y_idx < config_y_count_; y_idx++)
	{
		for(int x_idx=0; x_idx < config_x_count_; x_idx++)
		{
			for(int t_idx=0; t_idx < config_theta_count_; t_idx++)
			{
				int indx = y_idx*config_x_count_*config_theta_count_ + x_idx*config_theta_count_ + t_idx;
				float t_val = min_theta_ + t_idx*theta_resolution;
				float x_val = config_min_x + x_idx*x_resolution;
				float y_val = road_min_y_ + y_idx*y_resolution;
				vehicle_configs_.row(indx) = Eigen::Vector4f(x_val,y_val,t_val,0);
				//				std::cout<<"config at:"<< indx << "after update:" << vehicle_configs_.col(indx) << std::endl;
			}
		}
	}
	//	std::cout <<"rows:" << vehicle_configs_.rows() << "\t cols:" << vehicle_configs_.cols() << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRT<PointT>::generateQuasiRandomConfigurations ()
{			
	float x_resolution = tire_width_;
	float y_resolution = tire_width_;
	float theta_resolution = (max_theta_ - min_theta_)/(float)(config_theta_count_ - 1);

	float t_min_x = 3*road_min_x_;
	float t_max_x = 3*road_max_x_;
	float t_min_y = 2.5*road_min_y_;
	float t_max_y = 2.5*road_max_y_;

	int temp_x_configs = round((t_max_x - t_min_x)/(float)(x_resolution));
	int temp_y_configs = round((t_max_y - t_min_y)/(float)(y_resolution));
	//std::cout <<"temp_x_configs:" << temp_x_configs << "\t temp_y_configs:" << temp_y_configs << std::endl;

	Eigen::MatrixXf t_configs(2,temp_x_configs*temp_y_configs);
	for (int yidx = 0; yidx < temp_y_configs; yidx++)
	{
		for (int xidx = 0; xidx < temp_x_configs; xidx++)
		{
			int indx = yidx*temp_x_configs + xidx;
			float x_val = t_min_x + xidx*x_resolution;
			float y_val = t_min_y + yidx*y_resolution;
			t_configs.col(indx) = Eigen::Vector2f(x_val,y_val);
		}
	}
	//std::cout <<"t_configs rows:" << t_configs.rows() << "\t t_configs cols:" << t_configs.cols() << std::endl;

	int config_count = temp_x_configs * temp_y_configs * config_theta_count_;
	Eigen::MatrixXf vehicle_configs = Eigen::MatrixXf::Constant(config_count,4,-1000);

	int config_counter = 0;

	for (float theta = min_theta_; theta <= max_theta_; theta += theta_resolution)
	{
		Eigen::Matrix2f rotation_matrix;
		rotation_matrix << cos((theta-90)*PI_/180),-sin((theta-90)*PI_/180),sin((theta-90)*PI_/180),cos((theta-90)*PI_/180);
		Eigen::MatrixXf n_configs = rotation_matrix * t_configs;		
		//	std::cout << "\t theta:" << theta  <<"\n rotation_matrix:" << rotation_matrix << std::endl;

		for (int idx = 0; idx < n_configs.cols(); idx++)
		{
			float x_val = n_configs.col(idx)(0);
			float y_val = n_configs.col(idx)(1);

			if (x_val >= road_min_x_ && x_val <= road_max_x_ && y_val >= road_min_y_ && y_val <= road_max_y_)
			{
				vehicle_configs.row(config_counter) = Eigen::Vector4f(x_val,y_val,theta,0);
				config_counter++;
				//std::cout <<"x_val:" << x_val << "\t y_val:" << y_val << std::endl;
			}
		}
	}
	vehicle_configs_ = vehicle_configs.topRows(config_counter);

	std::cout <<"rows:" << vehicle_configs_.rows() << "\t cols:" << vehicle_configs_.cols() << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRT<PointT>::generateRandomConfigurations ()
{					
	int config_count = config_x_count_ * config_y_count_ * config_theta_count_;
	vehicle_configs_ = Eigen::MatrixXf::Constant(config_count,4,-1000);

	for (int i_idx=0; i_idx < config_count; i_idx++)
	{
		float x_val = (road_max_x_ - road_min_x_) * rand () / (RAND_MAX + 1.0f) + road_min_x_;
		float y_val = (road_max_y_ - road_min_y_) * rand () / (RAND_MAX + 1.0f) + road_min_y_;
		float t_val = (max_theta_ - min_theta_) * rand () / (RAND_MAX + 1.0f) + min_theta_;

		vehicle_configs_.row(i_idx) = Eigen::Vector4f(x_val,y_val,t_val,0);			
	}

	//	float x_resolution = (road_max_x_ - road_min_x_)/(double)(config_x_count_ - 1);
	//	std::cout <<"road_min_x_:" << road_min_x_ << "\t road_max_x_:" << road_max_x_ << "\t x_resolution:" << x_resolution << std::endl;

	//	for (int s_idx=0; s_idx < config_x_count_; s_idx++)
	//	{
	//		float x_val = road_min_x_ + s_idx*x_resolution;
	//		float y_val = road_min_y_;
	//		float t_val = (max_theta_ - min_theta_) * rand () / (RAND_MAX + 1.0f) + min_theta_;
	//
	//		vehicle_configs_.row(s_idx) = Eigen::Vector4f(x_val,y_val,t_val,0);	
	//	}
	//
	//	for (int i_idx=config_x_count_; i_idx < config_count-config_x_count_*config_theta_count_; i_idx++)
	//	{
	//		float x_val = (road_max_x_ - road_min_x_) * rand () / (RAND_MAX + 1.0f) + road_min_x_;
	//		float y_val = (road_max_y_ - road_min_y_) * rand () / (RAND_MAX + 1.0f) + road_min_y_;
	//		float t_val = (max_theta_ - min_theta_) * rand () / (RAND_MAX + 1.0f) + min_theta_;
	//
	//		vehicle_configs_.row(i_idx) = Eigen::Vector4f(x_val,y_val,t_val,0);			
	//	}
	//
	//	int x_idx = 0;
	//	for (int e_idx=config_count-config_x_count_; e_idx < config_count; e_idx++)
	//	{
	//		float x_val = road_min_x_ + x_idx*x_resolution;;
	//		float y_val = road_max_y_;
	//		float t_val = (max_theta_ - min_theta_) * rand () / (RAND_MAX + 1.0f) + min_theta_;
	//		x_idx++;
	//
	//		vehicle_configs_.row(e_idx) = Eigen::Vector4f(x_val,y_val,t_val,0);	
	//	}
	//	std::cout <<"rows:" << vehicle_configs_.rows() << "\t cols:" << vehicle_configs_.cols() << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::PRT<PointT>::collisionChecker (Eigen::MatrixXf vehicle_state)
{
	/*
	 *  Check if any of the tyres are outside road bounds.
	 */
	float tyres_min_x = vehicle_state.row(0).rightCols(4).minCoeff();
	float tyres_max_x = vehicle_state.row(0).rightCols(4).maxCoeff();
	if(tyres_min_x < road_min_x_ || tyres_max_x > road_max_x_)
	{
		//		std::cout << "tyre not on road." << std::endl;
		return true;
	}

	/*
	 *  Check tyres for collision.
	 */
	for (int tyre_id=0; tyre_id < 4; tyre_id++) 
	{
		pcl::PointXYZ tyre_point;
		tyre_point.x = vehicle_state(0,4+tyre_id);	
		tyre_point.y = vehicle_state(1,4+tyre_id);
		tyre_point.z = vehicle_state(2,4+tyre_id);
		//cout <<"tyre position \t"<<tyrePos <<endl;
		/*
		 * Find the cells in DEM under tyre.
		 */
		std::vector<int> dem_idxs;
		std::vector<float> dem_dists;
		dem_search_->radiusSearch(tyre_point,tire_width_/2,dem_idxs,dem_dists,0);

		//		std::cout <<"points under vehicle tyre \t"<<pointIdx.size() << std::endl;
		if(dem_idxs.size() == 0) // not enough points udner tyre
		{
			//			std::cout <<"vehicle state:\t"<<vehicle_state << std::endl;
			//			exit (EXIT_FAILURE);
			continue;
		}
		for (int d_idx=0; d_idx < dem_idxs.size(); d_idx++)
		{
			bool is_dem_collide = tyreClusterCollisionProcessing(dem_idxs[d_idx]);
			if(is_dem_collide)
			{
				//				std::cout << "tyre collision with negative obstacle." << std::endl;
				return true;
			}
		}
	}

	/*
	 *  Check vehicle body for collision.
	 */
	float state_min_x = vehicle_state.row(0).leftCols(4).minCoeff();
	float state_max_x = vehicle_state.row(0).leftCols(4).maxCoeff();
	float state_min_y = vehicle_state.row(1).leftCols(4).minCoeff();
	float state_max_y = vehicle_state.row(1).leftCols(4).maxCoeff();
	Eigen::Vector3f box_min_corner(state_min_x,state_min_y,-1);
	Eigen::Vector3f box_max_corner(state_max_x,state_max_y,1);

	//	std::cout << "state min corner: " << box_min_corner << "\t max corner:" << box_max_corner << std::endl;

	std::vector<int> dem_idxs;
	dem_search_->boxSearch (box_min_corner,box_max_corner,dem_idxs);
	//	std::cout << "point inside box are :"<< dem_idxs.size() << std::endl;
	for (int d_idx=0; d_idx < dem_idxs.size(); d_idx++)
	{
		bool is_dem_collide = bodyClusterCollisionProcessing(dem_idxs[d_idx]);
		if(is_dem_collide)
		{
			//			std::cout << "body collision with positive obstacle." << std::endl;
			bool is_dem_under_vehicle = checkDemUnderVehicle(dem_idxs[d_idx],vehicle_state.leftCols(4));
			if(is_dem_under_vehicle)
			{
				//				std::cout << "dem under vehicle body." << std::endl;
				return true;
			}else
			{
				//				std::cout << "dem NOT under vehicle body." << std::endl;
			}
		}
	}

	return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::PRT<PointT>::collisionAndSafetyChecker (Eigen::MatrixXf vehicle_state)
{
	/*
	 *  Check if any of the tyres are outside road bounds.
	 */
	float tyres_min_x = vehicle_state.row(0).rightCols(4).minCoeff();
	float tyres_max_x = vehicle_state.row(0).rightCols(4).maxCoeff();
	if(tyres_min_x < road_min_x_ || tyres_max_x > road_max_x_)
	{
		//std::cout << "tyre not on road." << std::endl;
		return -1;
	}

	/*
	 *  Check tyres for collision.
	 */
	for (int tyre_id=0; tyre_id < 4; tyre_id++) 
	{
		pcl::PointXYZ tyre_point;
		tyre_point.x = vehicle_state(0,4+tyre_id);	
		tyre_point.y = vehicle_state(1,4+tyre_id);
		tyre_point.z = vehicle_state(2,4+tyre_id);
		//std::cout <<"tyre position \t"<<tyre_point << std::endl;
		/*
		 * Find the cells in DEM under tyre.
		 */
		std::vector<int> dem_idxs;
		std::vector<float> dem_dists;
		float search_radius = tire_width_/2 + vehicle_safety_dist_;
		dem_search_->radiusSearch(tyre_point,search_radius,dem_idxs,dem_dists,0);

		//std::cout <<"points under vehicle tyre \t"<<dem_idxs.size() << std::endl;
		if(dem_idxs.size() == 0) // not enough points udner tyre
		{
			//			std::cout <<"vehicle state:\t"<<vehicle_state << std::endl;
			//			exit (EXIT_FAILURE);
			continue;
		}

		for (int d_idx=0; d_idx < dem_idxs.size(); d_idx++)
		{
			//int dem_idx = dem_idxs[d_idx];
			//Eigen::Vector3f dem_point(cloud_dem_->points[dem_idx].x, cloud_dem_->points[dem_idx].y, cloud_dem_->points[dem_idx].z);			
			bool is_dem_collide = tyreClusterCollisionProcessing(dem_idxs[d_idx]);
			//std::cout << "collision with negative obstacle :\t" << is_dem_collide << "\t due to dem:" <<  dem_point.transpose() << std::endl;
			if(is_dem_collide)
			{
				//std::cout << "tyre collision with negative obstacle." << std::endl;
				return -1;
			}
		}
	}

	/*
	 *  Check vehicle body for collision.
	 */
	float state_min_x = vehicle_state.row(0).leftCols(4).minCoeff();
	float state_max_x = vehicle_state.row(0).leftCols(4).maxCoeff();
	float state_min_y = vehicle_state.row(1).leftCols(4).minCoeff();
	float state_max_y = vehicle_state.row(1).leftCols(4).maxCoeff();
	Eigen::Vector3f box_min_corner(state_min_x,state_min_y,-1);
	Eigen::Vector3f box_max_corner(state_max_x,state_max_y,1);

	//	std::cout << "state min corner: " << box_min_corner << "\t max corner:" << box_max_corner << std::endl;

	std::vector<int> dem_idxs;
	dem_search_->boxSearch (box_min_corner,box_max_corner,dem_idxs);
	//	std::cout << "point inside box are :"<< dem_idxs.size() << std::endl;
	for (int d_idx=0; d_idx < dem_idxs.size(); d_idx++)
	{
		bool is_dem_collide = bodyClusterCollisionProcessing(dem_idxs[d_idx]);
		if(is_dem_collide)
		{
			//			std::cout << "body collision with positive obstacle." << std::endl;
			bool is_dem_under_vehicle = checkDemUnderVehicle(dem_idxs[d_idx],vehicle_state.leftCols(4));
			if(is_dem_under_vehicle)
			{
				//				std::cout << "dem under vehicle body." << std::endl;
				return -1;
			}else
			{
				//				std::cout << "dem NOT under vehicle body." << std::endl;
			}
		}
	}

	return 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::PRT<PointT>::computeConfigClearance (Eigen::Vector3f config, Eigen::MatrixXf vehicle_state)
{
	float clearance_value = 0;
	/*
	 *  get initial clearance as vehicle's corners distance from road sides
	 */
	//	float vehicle_min_x = vehicle_state.row(0).leftCols(4).minCoeff();
	//	float vehicle_max_x = vehicle_state.row(0).leftCols(4).maxCoeff();
	float vehicle_min_x = vehicle_state.row(0).rightCols(4).minCoeff();
	float vehicle_max_x = vehicle_state.row(0).rightCols(4).maxCoeff();

	float vehicle_dist1 = std::abs(road_max_x_ - std::abs(vehicle_min_x))/cos(PI_*(90-config(2))/(float)180);
	float vehicle_dist2 = std::abs(road_max_x_ - std::abs(vehicle_max_x))/cos(PI_*(90-config(2))/(float)180);

	if(vehicle_dist1 < vehicle_dist2)
	{
		clearance_value = vehicle_dist1;
	} else
	{
		clearance_value = vehicle_dist2;
	}

	if(clearance_value < 0)
	{
		clearance_value = 0;
		//	std::cout << "clearance due to tire on road edge:\t" << clearance_value << std::endl;
		return clearance_value;
	}
	//std::cout << "min distance bw tire and road edge:\t" << clearance_value << std::endl;
	/*
	 * get vehicle clearance bounding boxes 
	 */
	//	Eigen::MatrixXf clearance_bounding_boxes = getClearanceBoundingBoxes(Eigen::Vector3f config);

	/*
	 * check clearance of the config using dem cells surrounding vehicle
	 */
	/*
	 * Find center of config
	 */
	Eigen::Vector3f vechicle_center = vehicle_state.leftCols(4).rowwise().sum()/(float)4;	

	pcl::PointXYZ vehicle_center_point;
	vehicle_center_point.x = vechicle_center(0);
	vehicle_center_point.y = vechicle_center(1);
	vehicle_center_point.z = 0;
	/*
	 * Find the cells in DEM around the vehicle center using MAX_CLEARANCE_ as radius and sort based on distance from center
	 */
	std::vector<int> dem_idxs;
	std::vector<float> dem_dists;
	dem_search_->radiusSearch(vehicle_center_point,MAX_CLEARANCE_,dem_idxs,dem_dists,0);

	//std::cout << "dem surrouding vehicle:\t" << dem_idxs.size() << std::endl;

	std::vector< std::pair<float, int> > point_distances;
	std::pair<float, int> pair;
	int point_count = dem_idxs.size();
	point_distances.resize (point_count, pair);
	for (int p_idx=0; p_idx < point_count; p_idx++)
	{
		point_distances[p_idx].first = dem_dists[p_idx];
		point_distances[p_idx].second = dem_idxs[p_idx];
	}
	std::sort (point_distances.begin (), point_distances.end (), comparePair);

	/*
	 * check if collision occur and find distance of collision
	 */
	Eigen::MatrixXf clearance_bounding_boxes = getClearanceBoundingBoxes(config);
	//	std::ofstream myfile;
	//	myfile.open ("dem_info.txt");
	//	float dist = -1;
	for (int p_idx=0; p_idx < point_count; p_idx++)
	{
		int dem_idx = point_distances[p_idx].second;		
		// find distance of positive obstacle from vehicle
		bool is_body_collide = bodyClusterCollisionProcessing(dem_idx);
		//	Eigen::Vector3f dem_point(cloud_dem_->points[dem_idx].x, cloud_dem_->points[dem_idx].y, cloud_dem_->points[dem_idx].z);
		//	std::cout << "dem_idx:" << dem_idx << "\t dem dist:" << point_distances[p_idx].first << "\t is collide:\t" << is_body_collide << "\t dem:" <<  dem_point.transpose() << std::endl;
		if(is_body_collide) {
			float positive_obs_dist = computePositiveObstacleDistance(dem_idx, vehicle_state);
			//			dist = positive_obs_dist;
			//			bool is_under_vehicle = checkDemUnderVehicle(dem_idx, vehicle_state)
			//std::cout << "cleraance due to positive obstacle :\t" << positive_obs_dist << std::endl;
			//	Eigen::Vector3f dem_point(cloud_dem_->points[dem_idx].x, cloud_dem_->points[dem_idx].y, cloud_dem_->points[dem_idx].z);
			//std::cout << "cleraance due to positive obstacle :\t" << positive_obs_dist << "\t due to dem:" <<  dem_point.transpose() << std::endl;
			if(positive_obs_dist != -1 && positive_obs_dist < clearance_value)
			{
				clearance_value = positive_obs_dist;
				//				Eigen::Vector3f dem_point(cloud_dem_->points[dem_idx].x, cloud_dem_->points[dem_idx].y, cloud_dem_->points[dem_idx].z);
				//				std::cout << "cleraance due to positive obstacle :\t" << positive_obs_dist << "\t due to dem:" <<  dem_point.transpose() << std::endl;
				//return clearance_value;
			}
		}
		// find distance of negative obstacle from vehicle
		bool is_tire_collide = tyreClusterCollisionProcessing(dem_idx);
		if(is_tire_collide) {


			bool is_inside_tire_box = checkDemInsideTiresBoundingBoxes(dem_idx, clearance_bounding_boxes);
			if (!is_inside_tire_box)
			{
				continue;
			}
			float negative_obs_dist = computeNegativeObstacleDistance(dem_idx, vehicle_state);
			//			dist = negative_obs_dist;
			//	Eigen::Vector3f dem_point(cloud_dem_->points[dem_idx].x, cloud_dem_->points[dem_idx].y, cloud_dem_->points[dem_idx].z);
			//	std::cout << "cleraance due to negative obstacle :\t" << negative_obs_dist << "\t due to dem:" <<  dem_point.transpose() << std::endl;
			if(negative_obs_dist < clearance_value)
			{
				clearance_value = negative_obs_dist;
				//				Eigen::Vector3f dem_point(cloud_dem_->points[dem_idx].x, cloud_dem_->points[dem_idx].y, cloud_dem_->points[dem_idx].z);
				//				std::cout << "----cleraance due to negative obstacle :\t" << negative_obs_dist << "\t due to dem:" <<  dem_point.transpose() << std::endl;
			}
		}
		int collision_status = 0;
		if(is_body_collide)
		{
			collision_status = 1;
		}else if (is_tire_collide)
		{
			collision_status = -1;
		}
		//		Eigen::Vector3f dem_point(cloud_dem_->points[dem_idx].x, cloud_dem_->points[dem_idx].y, cloud_dem_->points[dem_idx].z);
		//		myfile << dem_point.transpose() <<" "<< collision_status <<" "<< dist << "\n";
	}
	//	myfile.close();
	return clearance_value;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::PRT<PointT>::checkDemUnderVehicle (int d_idx, Eigen::MatrixXf vehicle_body_state)
{
	/*
	 * A,B,C and D are corners of the vehicle state. let p be the point then the equation is (0 <= AP.AB <= AB.AB) && (0 <= AP.AD <= AD.AD)
	 */
	Eigen::Vector3f p(cloud_dem_->points[d_idx].x, cloud_dem_->points[d_idx].y, cloud_dem_->points[d_idx].z);
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::PRT<PointT>::checkDemInsideTiresBoundingBoxes (int d_idx, Eigen::MatrixXf clearance_bounding_boxes)
{
	/*
	 * A,B,C and D are corners of the vehicle tire bounding box. let p be the point then the equation is (0 <= AP.AB <= AB.AB) && (0 <= AP.AD <= AD.AD)
	 */
	for (int tire_id = 1; tire_id <=2; tire_id++)
	{
		Eigen::Vector3f p(cloud_dem_->points[d_idx].x, cloud_dem_->points[d_idx].y, cloud_dem_->points[d_idx].z);
		Eigen::Vector3f a_point = clearance_bounding_boxes.block(tire_id*3,0,3,1);
		Eigen::Vector3f b_point = clearance_bounding_boxes.block(tire_id*3,1,3,1);
		Eigen::Vector3f d_point = clearance_bounding_boxes.block(tire_id*3,3,3,1);

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
	}
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::PRT<PointT>::computePositiveObstacleDistance (int d_idx, Eigen::MatrixXf vehicle_body_state)
{
	Eigen::Vector3f dem_point(cloud_dem_->points[d_idx].x, cloud_dem_->points[d_idx].y, cloud_dem_->points[d_idx].z);
	//Eigen::Vector3f dem_point(0,0,0);
	//std::cout << "dem_point :\t" << dem_point << std::endl;
	float d1 = pointDistFromLine(dem_point,vehicle_body_state.col(0),vehicle_body_state.col(1));
	float d2 = pointDistFromLine(dem_point,vehicle_body_state.col(1),vehicle_body_state.col(2));
	float d3 = pointDistFromLine(dem_point,vehicle_body_state.col(2),vehicle_body_state.col(3));
	float d4 = pointDistFromLine(dem_point,vehicle_body_state.col(3),vehicle_body_state.col(0));

	//std::cout << "d1:" << d1 << "\t d2:" << d2 << "\t d3:" << d3 << "\t d4:" << d4 << std::endl;

	int width_far_counter = 0;
	int length_far_counter = 0;
	if (d1 > vehicle_length_) 
	{
		length_far_counter++;
	}
	if (d2 > vehicle_width_) 
	{
		width_far_counter++;
	}
	if (d3 > vehicle_length_) 
	{
		length_far_counter++;
	}
	if (d4 > vehicle_width_) 
	{
		width_far_counter++;
	}

	if(width_far_counter == 0 && length_far_counter == 0)
	{
		return 0;
	}
	else if(width_far_counter == 0 && length_far_counter != 0)
	{
		return -1;
		Eigen::Vector2f side_distances(d1,d3);
		float min_dist = side_distances.minCoeff();
		return min_dist;
	}
	else if(width_far_counter != 0 && length_far_counter == 0)
	{
		Eigen::Vector2f side_distances(d2,d4);
		float min_dist = side_distances.minCoeff();
		return min_dist;
	}else
	{
		return -1;
		float c1 = pointDistFromPoint(dem_point,vehicle_body_state.col(0));
		float c2 = pointDistFromPoint(dem_point,vehicle_body_state.col(1));
		float c3 = pointDistFromPoint(dem_point,vehicle_body_state.col(2));
		float c4 = pointDistFromPoint(dem_point,vehicle_body_state.col(3));

		//std::cout << "c1:" << c1 << "\t c2:" << c2 << "\t c3:" << c3 << "\t c4:" << c4 << std::endl;

		Eigen::Vector4f corner_distances(c1,c2,c3,c4);
		float min_dist = corner_distances.minCoeff();
		return min_dist;
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::PRT<PointT>::computeNegativeObstacleDistance (int d_idx, Eigen::MatrixXf vehicle_body_state)
{
	Eigen::Vector3f dem_point(cloud_dem_->points[d_idx].x, cloud_dem_->points[d_idx].y, cloud_dem_->points[d_idx].z);
	float c1 = pointDistFromPoint(dem_point,vehicle_body_state.col(4));
	float c2 = pointDistFromPoint(dem_point,vehicle_body_state.col(5));
	float c3 = pointDistFromPoint(dem_point,vehicle_body_state.col(6));
	float c4 = pointDistFromPoint(dem_point,vehicle_body_state.col(7));

	Eigen::Vector4f tire_distances(c1,c2,c3,c4);
	float min_dist = tire_distances.minCoeff();

	//	if(dem_point(0)>0.387 && dem_point(0)<0.3872 && dem_point(1)>1.385 && dem_point(1)<1.387)
	//	{
	//		std::cout << "vehicle_body_state \n" << vehicle_body_state << std::endl;
	//		std::cout << "c1:" << c1 << "\t c2:" << c2 << "\t c3:" << c3 << "\t c4:" << c4 << "\t min:" << min_dist << std::endl;
	//	}

	return (min_dist-tire_width_/2);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//template <typename PointT> bool
//pcl::PRT<PointT>::checkDemUnderVehicle (int d_idx, Eigen::MatrixXf vehicle_body_state)
//{
//	Eigen::Vector3f dem_point(cloud_dem_->points[d_idx].x, cloud_dem_->points[d_idx].y, cloud_dem_->points[d_idx].z);
//	double d1 = pointDistFromLine(dem_point,vehicle_body_state.col(0),vehicle_body_state.col(1));
//	if (d1 > vehicle_length_) 
//	{
//		//		std::cout << "body collision with positive obstacle." << std::endl;
//		return false;
//	}
//	double d2 = pointDistFromLine(dem_point,vehicle_body_state.col(1),vehicle_body_state.col(2));
//	if (d2 > vehicle_width_) 
//	{
//		//		std::cout << "body collision with positive obstacle." << std::endl;
//		return false;
//	}
//	double d3 = pointDistFromLine(dem_point,vehicle_body_state.col(2),vehicle_body_state.col(3));
//	if (d3 > vehicle_length_) 
//	{
//		//		std::cout << "body collision with positive obstacle." << std::endl;
//		return false;
//	}
//	double d4 = pointDistFromLine(dem_point,vehicle_body_state.col(3),vehicle_body_state.col(0));
//	if (d4 > vehicle_width_) 
//	{
//		//		std::cout << "body collision with positive obstacle." << std::endl;
//		return false;
//	}
//	return true;
//}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::PRT<PointT>::tyreClusterCollisionProcessing (int d_idx)
{
	std::vector<cluster_stats_> dem_cluster = dem_clusters_[d_idx];
	//	std::cout << "dem_cluster.size():\t" << dem_cluster.size() << std::endl;
	if(dem_cluster.size() == 0)
	{
		return false;
	}

	bool is_above_ground = false;
	for (int c_idx=dem_cluster.size()-1; c_idx >= 0; c_idx--)
	{
		//std::cout << "dem_cluster[c_idx].min_dist:" << dem_cluster[c_idx].min_dist << "\t dem_cluster[c_idx].max_dist:" << dem_cluster[c_idx].max_dist << std::endl;
		if ((dem_cluster[c_idx].min_dist >= 0 && dem_cluster[c_idx].min_dist < ground_clearance_) && (dem_cluster[c_idx].max_dist >= 0 && dem_cluster[c_idx].max_dist < ground_clearance_))
		{
			is_above_ground = true;
		}

		if (dem_cluster[c_idx].max_dist >= ground_clearance_ && dem_cluster[c_idx].max_dist <= vehicle_heigt_)
		{
			//					std::cout << "points under tyre collide with vehicle body: max distance." << std::endl;
			return true;
		}
		if(dem_cluster[c_idx].min_dist >= ground_clearance_ && dem_cluster[c_idx].min_dist <= vehicle_heigt_)
		{
			//				std::cout << "points under tyre collide with vehicle body: min distance." << std::endl;
			return true;
		}
		if (dem_cluster[c_idx].max_dist < negative_obs_threshold_ &&  is_above_ground == false)
		{
			//					std::cout << "points under tyre collide fall in negative obstacle." << std::endl;
			return true;
		}
	}

	return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::PRT<PointT>::bodyClusterCollisionProcessing (int d_idx)
{
	std::vector<cluster_stats_> dem_cluster = dem_clusters_[d_idx];
	if(dem_cluster.size() == 0)
	{
		return false;
	}

	for (int c_idx=dem_cluster.size()-1; c_idx >= 0; c_idx--)
	{
		//		if(dem_cluster[c_idx].min_dist !=0)
		//			std::cout <<"size:" << dem_cluster[c_idx].cluster_size << "\t min:" << dem_cluster[c_idx].min_dist << "\t max:" << dem_cluster[c_idx].max_dist << std::endl;
		if (dem_cluster[c_idx].max_dist > ground_clearance_ && dem_cluster[c_idx].max_dist < vehicle_heigt_)
		{
			return true;
		}
		if(dem_cluster[c_idx].min_dist > ground_clearance_ && dem_cluster[c_idx].min_dist < vehicle_heigt_)
		{
			return true;
		}
	}

	return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> Eigen::MatrixXf
pcl::PRT<PointT>::getVehicleState (Eigen::Vector3f v_config)
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> Eigen::MatrixXf
pcl::PRT<PointT>::getVehicleStateWithClearance (Eigen::Vector3f v_config)
{
	Eigen::MatrixXf vehicle_state_origin(3,8);
	vehicle_state_origin << -vehicle_width_/2-vehicle_safety_dist_,vehicle_width_/2+vehicle_safety_dist_,vehicle_width_/2+vehicle_safety_dist_,-vehicle_width_/2-vehicle_safety_dist_,-front_track_/2,front_track_/2, front_track_/2, -front_track_/2,
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> Eigen::MatrixXf
pcl::PRT<PointT>::getClearanceBoundingBoxes (Eigen::Vector3f v_config)
{
	Eigen::MatrixXf front_wheels_box(4,4);
	front_wheels_box << -front_track_/2-tire_width_/2-MAX_CLEARANCE_,front_track_/2+tire_width_/2+MAX_CLEARANCE_,front_track_/2+tire_width_/2+MAX_CLEARANCE_,-front_track_/2-tire_width_/2-MAX_CLEARANCE_,
			wheelbase_-tire_width_/2,wheelbase_-tire_width_/2,wheelbase_+tire_width_/2,wheelbase_+tire_width_/2,
			0,0,0,0,1,1,1,1;

	Eigen::MatrixXf back_wheels_box(4,4);
	back_wheels_box << -front_track_/2-tire_width_/2-MAX_CLEARANCE_,front_track_/2+tire_width_/2+MAX_CLEARANCE_,front_track_/2+tire_width_/2+MAX_CLEARANCE_,-front_track_/2-tire_width_/2-MAX_CLEARANCE_,
			-tire_width_/2,-tire_width_/2,tire_width_/2,tire_width_/2,
			0,0,0,0,1,1,1,1;

	Eigen::MatrixXf vehicle_body_box(4,4);
	vehicle_body_box << -vehicle_width_/2-MAX_CLEARANCE_,vehicle_width_/2+MAX_CLEARANCE_,vehicle_width_/2+MAX_CLEARANCE_,-vehicle_width_/2-MAX_CLEARANCE_,
			-vehicle_length_/2+wheelbase_/2,-vehicle_length_/2+wheelbase_/2,vehicle_length_/2+wheelbase_/2,vehicle_length_/2+wheelbase_/2,
			0,0,0,0,1,1,1,1;

	/*
	 *  Transform using the configuration
	 */
	Eigen::Vector3f normal_xy_plane (0.0,0.0,1.0);
	Eigen::AngleAxis<float> rotation((PI_*(v_config(2)-90))/(float)180,normal_xy_plane);
	Eigen::Matrix3f rotation_matrix;
	rotation_matrix = rotation.matrix();	
	Eigen::Vector3f translation_vector (v_config(0),v_config(1),1);	
	Eigen::MatrixXf transform_matrix(3,4);
	transform_matrix << rotation_matrix , translation_vector;

	Eigen::MatrixXf front_wheels_box_transformed = transform_matrix*front_wheels_box;
	Eigen::MatrixXf back_wheels_box_transformed = transform_matrix*back_wheels_box;
	Eigen::MatrixXf vehicle_body_box_transformed = transform_matrix*vehicle_body_box;

	Eigen::MatrixXf clearance_bounding_boxes(9,4);
	clearance_bounding_boxes << front_wheels_box_transformed, back_wheels_box_transformed, vehicle_body_box_transformed;


	return clearance_bounding_boxes;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRT<PointT>::computePRT ()
{
	bool computation_is_possible = initCompute ();
	if ( !computation_is_possible )
	{
		deinitCompute ();
		return;
	}

	bool is_ready = preProcessingPRT();
	if(!is_ready)
	{
		std::cout << "Error, issue in pre-processing for PRT." << std::endl;
		return;
	}

	std::cout << "is_import_configs_ flag " << is_import_configs_<< std::endl;

	if (is_import_configs_)
	{
		std::cout << "Configs imported with count:" << vehicle_configs_.rows() << std::endl;
	} else
	{
		if (is_random_configs_)
		{
			generateRandomConfigurations();
			//generateQuasiRandomConfigurations();
			std::cout << "Generated random  configs." << std::endl;
		} else 
		{
			generateQuasiRandomConfigurations();
			//			generateConfigurations();
			std::cout << "Generate Quasi  static configs." << std::endl;
		}
	}
	//	generateRandomConfigurations();

	std::cout << "Validate Configs." << std::endl;
	validateConfigurations();

	std::cout << "Get Config Cloud." << std::endl;
	getConfigCloud();
	config_search_->setInputCloud(cloud_config_);
	std::cout << "Config Cloud has points." << cloud_config_->points.size() << std::endl;

	std::cout << "Find neighbors of configs." << std::endl;
	findConfigNeighbours();

	std::cout << "Generating Graph." << std::endl;
	generatePRTGraph();

	deinitCompute ();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::PRT<PointT>::preProcessingPRT ()
{
	std::cout << "Pre-processing for PRT." << std::endl;
	configs_limit_ = config_x_count_ * config_y_count_ * config_theta_count_;
	//	vehicle_configs_ = Eigen::MatrixXf::Constant(configs_limit_,4,-1000);
	// if user didn't set search method
	if (!dem_search_)
		dem_search_.reset (new pcl::octree::OctreePointCloudSearch<PointT>(128.0f));

	if (!config_search_)
		config_search_.reset (new pcl::search::KdTree<PointT>);
	//std::cout << "Points in cloud_dem_ before computeDEM(): " << cloud_dem_->points.size() << std::endl;
	computeDEM();

	std::cout << "Points in cloud_dem_ after computeDEM(): " << cloud_dem_->points.size() << std::endl;
	dem_search_->setInputCloud(cloud_dem_);
	dem_search_->addPointsFromInputCloud ();

	return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//template <typename PointT> pcl::PointCloud<pcl::PointXYZRGB>::Ptr
//pcl::PRT<PointT>::getPRTVisibilityCloud ()
//{
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
//	colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();
//
//	for (int i=0; i < config_collision_status_.size(); i++)
//	{
//		pcl::PointXYZRGB point;
//		point.x = vehicle_configs_.col(i)[0];
//		point.y = vehicle_configs_.col(i)[1];
//		point.z = 0;
//		point.r = 0;
//		point.g = 0;
//		point.b = 0;
//
//		if (config_collision_status_[i] == -1)
//		{
//			point.b = 255;
//		} else if (config_collision_status_[i] == 0)
//		{
//			point.r = 255;
//		} else
//		{
//			point.g = 255;
//		}
//		colored_cloud->points.push_back (point);
//	}
//
//	return (colored_cloud);
//}

#define PCL_INSTANTIATE_PRT(T) template class pcl::PRT<T>;

#endif /* PRT_HPP_ */

