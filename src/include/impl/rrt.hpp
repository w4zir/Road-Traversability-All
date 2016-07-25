/*
 * height_estimate.hpp
 *
 *  Created on: Dec 1, 2015
 *      Author: mudassir
 */

#ifndef RRT_HPP_
#define RRT_HPP_

#include <rrt.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::RRT<PointT>::RRT () :
configs_limit_ (5000),
start_configs_ (20),
config_counter_ (0),
min_theta_ (60),
max_theta_ (120),
vehicle_velocity_ (0.5),
vehicle_phi_ (12),
vehicle_width_ (1.58), //2
vehicle_length_ (2.2), //3
vehicle_heigt_ (1.9), //1.5
wheelbase_ (1.78), //2.2
front_track_ (1.3), //1.4
ground_clearance_ (0.25), //0.4
tyre_width_ (0.24), //0.2
negative_obs_threshold_ (0.2), //0.2
config_search_radius_ (0.3), //0.3
sub_config_count_(3), //2
vehicle_safety_dist_ (0.2),
goal_tolerance_ (1.0),
goal_bias_ (0.05),
vehicle_configs_ (),
config_collision_status_ (),
vehicle_state_ (),
dem_search_ (),
rrt_graph_ ()
{
	//search_ =  typename pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::RRT<PointT>::~RRT ()
{
	if (dem_search_ != 0)
		dem_search_.reset ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RRT<PointT>::getRRTAdjacencyList (std::vector< std::pair<int,int> >& rrt_graph)
{
	rrt_graph = rrt_graph_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RRT<PointT>::setGoalBiasProbability (float goal_bias)
{
	goal_bias_ = goal_bias;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::RRT<PointT>::getGoalBiasProbability () const
{
	return (goal_bias_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::RRT<PointT>::getConfigCounter () const
{
	return (config_counter_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//template <typename PointT> std::vector<int> 
//pcl::RRT<PointT>::getConfigurationValidityStatus ()
//{
//	return config_collision_status_;
//}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> Eigen::MatrixXf
pcl::RRT<PointT>::getConfigurations ()
{
	return vehicle_configs_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> Eigen::Vector3f
pcl::RRT<PointT>::generateRandomConfiguration ()
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
pcl::RRT<PointT>::findQrandNearestNeighbor (Eigen::Vector3f qrand)
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
pcl::RRT<PointT>::config_dist (Eigen::Vector3f qrand, Eigen::Vector4f qnear)
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
pcl::RRT<PointT>::findOptimalConfig (Eigen::Vector3f qrand, Eigen::Vector4f qnear)
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
		float phi = qnear(3) + direction*vehicle_phi_;
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
pcl::RRT<PointT>::checkGoalReached (Eigen::Vector4f q_new)
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
pcl::RRT<PointT>::checkConfigSafety (Eigen::Vector4f config)
{
	Eigen::MatrixXf vehicle_state = getVehicleStateWithClearance(config.head(3));
	//		std::cout << "vehicle state: " << vehicle_state << std::endl;
	//		float state_min_x = vehicle_state.row(1).leftCols(4).minCoeff();
	int config_safe = collisionAndSafetyChecker(vehicle_state);
	return (config_safe);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::RRT<PointT>::addStartConfigurations ()
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
pcl::RRT<PointT>::generateRRTGraph ()
{
	std::vector< std::pair<int,int> > rrt_graph;
	//float phi_step = vehicle_phi_ / (float)(sub_config_count_ + 1);

	bool is_goal = false;	
	int counter = config_counter_;
	std::cout << "config count at start of generateRRTGraph ():"<< config_counter_ << std::endl;
	while (!is_goal && counter < configs_limit_)
	{
		Eigen::Vector3f q_rand = generateRandomConfiguration();
		int q_near_idx = findQrandNearestNeighbor(q_rand);
		Eigen::Vector4f q_new = findOptimalConfig(q_rand,vehicle_configs_.row(q_near_idx));
//		std::cout <<"Optimal config:"<< q_new.transpose() << std::endl;

		int is_q_safe = checkConfigSafety(q_new);
		if(is_q_safe == 1)
		{			
			vehicle_configs_.row(config_counter_) = q_new;
			rrt_graph.push_back(std::make_pair(q_near_idx,config_counter_));			
			is_goal = checkGoalReached(q_new);
			config_counter_++;
		}
		counter++;
		//std::cout << "config counter:" << counter << "\t safe counter:" << config_counter_ <<"\t q:"<<q_new.transpose()<< std::endl;
	}
	std::cout << "config counter:" << counter << "\t safe counter:" << config_counter_ << std::endl;
	rrt_graph_ = rrt_graph;
	std::cout << "RRT graph generated." << std::endl;
	if(is_goal)
	{
		std::cout << "Goal reached successfully !!!" << std::endl;
	}else
	{
		std::cout << "Could not reach goal :(" << std::endl;
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::RRT<PointT>::collisionChecker (Eigen::MatrixXf vehicle_state)
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
		dem_search_->radiusSearch(tyre_point,tyre_width_/2,dem_idxs,dem_dists,0);

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
pcl::RRT<PointT>::collisionAndSafetyChecker (Eigen::MatrixXf vehicle_state)
{
	/*
	 *  Check if any of the tyres are outside road bounds.
	 */
	float tyres_min_x = vehicle_state.row(0).rightCols(4).minCoeff();
	float tyres_max_x = vehicle_state.row(0).rightCols(4).maxCoeff();
	if(tyres_min_x < road_min_x_ || tyres_max_x > road_max_x_)
	{
		//		std::cout << "tyre not on road." << std::endl;
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
		//cout <<"tyre position \t"<<tyrePos <<endl;
		/*
		 * Find the cells in DEM under tyre.
		 */
		std::vector<int> dem_idxs;
		std::vector<float> dem_dists;
		float search_radius = tyre_width_/2 + vehicle_safety_dist_;
		dem_search_->radiusSearch(tyre_point,search_radius,dem_idxs,dem_dists,0);

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
template <typename PointT> bool
pcl::RRT<PointT>::checkDemUnderVehicle (int d_idx, Eigen::MatrixXf vehicle_body_state)
{
	Eigen::Vector3f dem_point(cloud_dem_->points[d_idx].x, cloud_dem_->points[d_idx].y, cloud_dem_->points[d_idx].z);
	double d1 = pointDistFromLine(dem_point,vehicle_body_state.col(0),vehicle_body_state.col(1));
	double d2 = pointDistFromLine(dem_point,vehicle_body_state.col(1),vehicle_body_state.col(2));
	double d3 = pointDistFromLine(dem_point,vehicle_body_state.col(2),vehicle_body_state.col(3));
	double d4 = pointDistFromLine(dem_point,vehicle_body_state.col(3),vehicle_body_state.col(0));
	if (d1 < vehicle_length_ && d2 < vehicle_width_ && d3 < vehicle_length_ &&  d4 < vehicle_width_) 
	{
		//		std::cout << "body collision with positive obstacle." << std::endl;
		return true;
	}
	return false;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::RRT<PointT>::tyreClusterCollisionProcessing (int d_idx)
{
	std::vector<cluster_stats_> dem_cluster = dem_clusters_[d_idx];
	if(dem_cluster.size() == 0)
	{
		return false;
	}

	bool is_above_ground = false;
	for (int c_idx=dem_cluster.size()-1; c_idx >= 0; c_idx--)
	{
		if ((dem_cluster[c_idx].min_dist >= 0 && dem_cluster[c_idx].min_dist < ground_clearance_) && (dem_cluster[c_idx].max_dist >= 0 && dem_cluster[c_idx].max_dist < ground_clearance_))
		{
			is_above_ground = true;
		}

		if (dem_cluster[c_idx].max_dist >= ground_clearance_ && dem_cluster[c_idx].max_dist <= vehicle_heigt_)
		{
			//				std::cout << "points under tyre collide with vehicle body: max distance." << std::endl;
			return true;
		}
		if(dem_cluster[c_idx].min_dist >= ground_clearance_ && dem_cluster[c_idx].min_dist <= vehicle_heigt_)
		{
			//				std::cout << "points under tyre collide with vehicle body: min distance." << std::endl;
			return true;
		}
		if (dem_cluster[c_idx].max_dist < negative_obs_threshold_ &&  is_above_ground == false)
		{
			//				std::cout << "points under tyre collide fall in negative obstacle." << std::endl;
			return true;
		}
	}

	return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::RRT<PointT>::bodyClusterCollisionProcessing (int d_idx)
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
pcl::RRT<PointT>::getVehicleState (Eigen::Vector3f v_config)
{
	Eigen::MatrixXf vehicle_state_origin(3,8);
	vehicle_state_origin << -vehicle_width_/2,vehicle_width_/2,vehicle_width_/2,-vehicle_width_/2,-front_track_/2,front_track_/2, front_track_/2, -front_track_/2,
			-vehicle_length_/2+wheelbase_/2,-vehicle_length_/2+wheelbase_/2,vehicle_length_/2+wheelbase_/2,vehicle_length_/2+wheelbase_/2,0,0,wheelbase_,wheelbase_,
			0,0,0,0,0,0,0,0;

	/*
	 *  Find the vehicle position using its configuration.
	 */
	Eigen::Vector3f normal_xy_plane (0.0,0.0,1.0);
	Eigen::AngleAxis<float> transform((PI_*cos(90-v_config(2)))/180,normal_xy_plane);
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
pcl::RRT<PointT>::getVehicleStateWithClearance (Eigen::Vector3f v_config)
{
	Eigen::MatrixXf vehicle_state_origin(3,8);
	vehicle_state_origin << -vehicle_width_/2-vehicle_safety_dist_,vehicle_width_/2+vehicle_safety_dist_,vehicle_width_/2+vehicle_safety_dist_,-vehicle_width_/2-vehicle_safety_dist_,-front_track_/2,front_track_/2, front_track_/2, -front_track_/2,
			-vehicle_length_/2+wheelbase_/2,-vehicle_length_/2+wheelbase_/2,vehicle_length_/2+wheelbase_/2,vehicle_length_/2+wheelbase_/2,0,0,wheelbase_,wheelbase_,
			0,0,0,0,0,0,0,0;

	/*
	 *  Find the vehicle position using its configuration.
	 */
	Eigen::Vector3f normal_xy_plane (0.0,0.0,1.0);
	Eigen::AngleAxis<float> transform((PI_*cos(90-v_config(2)))/180,normal_xy_plane);
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
template <typename PointT> void
pcl::RRT<PointT>::computeRRT ()
{
	bool computation_is_possible = initCompute ();
	if ( !computation_is_possible )
	{
		deinitCompute ();
		return;
	}

	bool is_ready = preProcessingRRT();
	if(!is_ready)
	{
		std::cout << "Error, issue in pre-processing for RRT." << std::endl;
		return;
	}

	std::cout << "Adding start configurations." << std::endl;
	addStartConfigurations();

	std::cout << "Generating RRT Graph." << std::endl;
	generateRRTGraph();	

	deinitCompute ();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::RRT<PointT>::preProcessingRRT ()
{
	std::cout << "Pre-processing for RRT." << std::endl;
	vehicle_configs_ = Eigen::MatrixXf::Constant(configs_limit_,4,-1000);
	// if user didn't set search method
	if (!dem_search_)
		dem_search_.reset (new pcl::octree::OctreePointCloudSearch<PointT>(128.0f));

	//std::cout << "Points in cloud_dem_ before computeDEM(): " << cloud_dem_->points.size() << std::endl;
	computeDEM();

	std::cout << "Points in cloud_dem_ after computeDEM(): " << cloud_dem_->points.size() << std::endl;
	dem_search_->setInputCloud(cloud_dem_);
	dem_search_->addPointsFromInputCloud ();
	return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//template <typename PointT> pcl::PointCloud<pcl::PointXYZRGB>::Ptr
//pcl::RRT<PointT>::getRRTVisibilityCloud ()
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

#define PCL_INSTANTIATE_RRT(T) template class pcl::RRT<T>;

#endif /* RRT_HPP_ */
