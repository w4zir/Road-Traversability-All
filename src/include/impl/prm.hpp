/*
 * height_estimate.hpp
 *
 *  Created on: Dec 1, 2015
 *      Author: mudassir
 */

#ifndef PRM_HPP_
#define PRM_HPP_

#include <prm.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::PRM<PointT>::PRM () :
config_x_count_ (10),
config_y_count_ (50),
config_theta_count_ (7),
min_theta_ (60),
max_theta_ (120),
vehicle_width_ (1.58), //2
vehicle_length_ (2.2), //3
vehicle_heigt_ (1.9), //1.5
wheelbase_ (1.78), //2.2
front_track_ (1.3), //1.4
ground_clearance_ (0.25), //0.4
tyre_width_ (0.24), //0.2
negative_obs_threshold_ (0.2), //0.2
config_search_radius_ (0.3), //0.3
sub_config_count_(2), //2
is_random_config_(false),
dem_search_ (),
cloud_prm_ (),
vehicle_configs_ (),
config_collision_status_ (),
vehicle_state_ (),
prm_graph_ (),
default_dem_cells_ ()
{
	//search_ =  typename pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::PRM<PointT>::~PRM ()
{
	if (dem_search_ != 0)
		dem_search_.reset ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::PRM<PointT>::getConfigurationX () const
{
	return (config_x_count_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRM<PointT>::setConfigurationX (int config_x_count)
{
	config_x_count_ = config_x_count;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::PRM<PointT>::getConfigurationY () const
{
	return (config_y_count_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRM<PointT>::setConfigurationY (int config_y_count)
{
	config_y_count_ = config_y_count;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRM<PointT>::setConfigurationTheta (int config_theta_count)
{
	config_theta_count_ = config_theta_count;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::PRM<PointT>::getConfigurationTheta () const
{
	return (config_theta_count_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRM<PointT>::getPRMAdjacencyList (std::vector< std::pair<int,int> >& prm_graph)
{
	prm_graph = prm_graph_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRM<PointT>::setRandomConfigFlag (bool random_config_flag)
{
	is_random_config_ = random_config_flag;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::PRM<PointT>::getRandomConfigFlag () const
{
	return (is_random_config_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> std::vector<int> 
pcl::PRM<PointT>::getConfigurationValidityStatus ()
{
	return config_collision_status_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> Eigen::MatrixXf
pcl::PRM<PointT>::getConfigurations ()
{
	return vehicle_configs_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRM<PointT>::setDEMSearchMethod (const OcTreePtr& tree)
{
	if (dem_search_ != 0)
		dem_search_.reset ();

	dem_search_ = tree;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRM<PointT>::getConfigCloud ()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr prm_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	for (int i=0; i < vehicle_configs_.cols(); i++)
	{
		pcl::PointXYZ point;
		point.x = vehicle_configs_.col(i)[0];
		point.y = vehicle_configs_.col(i)[1];
		point.z = 0;
		prm_cloud->push_back(point);
	}
	cloud_prm_ = prm_cloud;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> std::vector<int> 
pcl::PRM<PointT>::getTempNeighbors (int c_idx)
{
	std::vector<int> n_config_idx;
	Eigen::Vector3f c_config(vehicle_configs_.col(c_idx));
	float x_resolution = (road_max_x_ - road_min_x_)/(double)(config_x_count_ - 1);
	float y_resolution = (road_max_y_ - road_min_y_)/(double)(config_y_count_ - 1);
	//	float theta_resolution = (max_theta_ - min_theta_)/(double)(config_theta_count_ - 1);

	int xt = config_x_count_*config_theta_count_;
	int t = config_theta_count_;

	int arr[] = {c_idx+xt-t, c_idx+xt, c_idx+xt+t};
	n_config_idx.insert(n_config_idx.end(), arr, arr+(sizeof(arr)/sizeof(arr[0])));
	//	if(c_config[2] == min_theta_)
	//	{
	////		int arr[] = {c_idx+xt+t, c_idx+2*xt+t+1, c_idx+2*xt+t+2};
	//		int arr[] = {c_idx+xt-t, c_idx+xt, c_idx+xt+t};
	//		n_config_idx.insert(n_config_idx.end(), arr, arr+(sizeof(arr)/sizeof(arr[0])));
	//
	//	}else if(c_config[2] == (min_theta_+theta_resolution))
	//	{
	//		int arr[] = {c_idx+2*xt+t, c_idx+2*xt+t+1, c_idx+2*xt+2};
	//		n_config_idx.insert(n_config_idx.end(), arr, arr+(sizeof(arr)/sizeof(arr[0])));
	//	}else if(c_config[2] == (min_theta_+ 2*theta_resolution))
	//	{
	//		int arr[] = {c_idx+xt, c_idx+xt+t-2, c_idx+2*xt+t-1, c_idx+2*xt-t+1, c_idx+xt-t+2};
	//		n_config_idx.insert(n_config_idx.end(), arr, arr+(sizeof(arr)/sizeof(arr[0])));
	//	}else if(c_config[2] == (min_theta_+ 3*theta_resolution))
	//	{
	//		int arr[] = {c_idx+2*xt-t, c_idx+2*xt-t-1, c_idx+2*xt-2};
	//		n_config_idx.insert(n_config_idx.end(), arr, arr+(sizeof(arr)/sizeof(arr[0])));
	//	}else if(c_config[2] == (min_theta_+ 4*theta_resolution))
	//	{
	//		int arr[] = {c_idx+xt-t, c_idx+2*xt-t-1, c_idx+2*xt-t-2};
	//		n_config_idx.insert(n_config_idx.end(), arr, arr+(sizeof(arr)/sizeof(arr[0])));
	//	}

	return n_config_idx;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::PRM<PointT>::findConfigsConnectivity (int c_idx, int n_idx)
{	
	int config_count = config_x_count_ * config_y_count_ * config_theta_count_;
	if(n_idx > config_count)
	{
		return false;
	}

	Eigen::Vector3f c_config(vehicle_configs_.col(c_idx));
	Eigen::Vector3f n_config(vehicle_configs_.col(n_idx));

	Eigen::Vector3f config_diff = n_config - c_config;

	float angle_difference = (max_theta_ - min_theta_)/(config_theta_count_ - 1);
	//	if((n_config[0] - c_config[0]) <= 0 || (n_config[1] - c_config[1]) <= 0 || abs(n_config[2] - c_config[2]) > angle_difference)
	//	if(config_diff[1] <= 0 || abs(config_diff[2]) > angle_difference)
	//	{
	//		std::cout <<"no connectivity due to constraints." << config_diff.transpose()<< std::endl;
	//		return false;
	//	}

	//	x_new = cos ( c_config[2] * PI / 180.0 );
	//	y_new = sin ( c_config[2] * PI / 180.0 );
	//	if((x_new < 0 && config_diff[2] || abs(config_diff[2]) > angle_difference)
	//		{
	//			std::cout <<"config unreachable due to angle." << config_diff.transpose()<< std::endl;
	//			return false;
	//		}

	//	float xInc = (config_diff[0])/(sub_config_count_ + 1);
	//	float yInc = (config_diff[1])/(sub_config_count_ + 1);
	//	float zInc = 0;
	//	float thetaInc = (config_diff[2])/(sub_config_count_ + 1);
	// check sub points between two neibouring nodes for validity
	for (int sc_idx=1; sc_idx <= sub_config_count_; sc_idx++) 
	{
		Eigen::Vector3f sub_config(c_config + sc_idx*config_diff/(sub_config_count_ + 1));
		Eigen::MatrixXf vehicle_state = getVehicleState(sub_config);
		//		std::cout << "vehicle state: " << vehicle_state << std::endl;
		//		float state_min_x = vehicle_state.row(1).leftCols(4).minCoeff();
		bool config_valid_flag = collisionChecker(vehicle_state);
		//		std::cout << "sub-config:" << sub_config.transpose() << "\t validity:" << config_valid_flag << std::endl;
		if(config_valid_flag) 
		{
			return false;
		}
	}
	//	std::cout <<"----------------------------------" << std::endl;
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRM<PointT>::generateConfigurations ()
{					
	int config_count = config_x_count_ * config_y_count_ * config_theta_count_;
	//	std::cout<<"x:"<< config_x_count_ <<"\t y:"<< config_y_count_ <<"\t t:"<< config_theta_count_ <<"\t total:"<< config_count<< std::endl;
	vehicle_configs_ = Eigen::MatrixXf::Constant(3,config_count,-1);

	float config_min_x = road_min_x_;
	float config_max_x = road_max_x_;
	//	float config_min_x = road_min_x_ + tyre_width_/2 + front_track_/2;
	//		float config_max_x = road_max_x_ - tyre_width_/2 - front_track_/2;

	float x_resolution = (config_max_x - config_min_x)/(double)(config_x_count_ - 1);
	float y_resolution = (road_max_y_ - road_min_y_)/(double)(config_y_count_ - 1);
	float theta_resolution = 1;//(max_theta_ - min_theta_)/(double)(config_theta_count_ - 1);
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
				vehicle_configs_.col(indx) = Eigen::Vector3f(x_val,y_val,t_val);
				//				std::cout<<"config at:"<< indx << "after update:" << vehicle_configs_.col(indx) << std::endl;
			}
		}
	}
	//	std::cout <<"rows:" << vehicle_configs_.rows() << "\t cols:" << vehicle_configs_.cols() << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRM<PointT>::generateRandomConfigurations ()
{					
	int config_count = config_x_count_ * config_y_count_ * config_theta_count_;
	vehicle_configs_ = Eigen::MatrixXf::Constant(3,config_count,-1);

	float x_resolution = (road_max_x_ - road_min_x_)/(double)(config_x_count_ - 1);

	for (int s_idx=0; s_idx < config_x_count_; s_idx++)
	{
		float x_val = road_min_x_ + s_idx*x_resolution;;
		float y_val = road_min_y_;
		float t_val = 90;

		vehicle_configs_.col(s_idx) = Eigen::Vector3f(x_val,y_val,t_val);	
	}

	for (int i_idx=config_x_count_; i_idx < config_count-config_x_count_; i_idx++)
	{
		float x_val = (road_max_x_ - road_min_x_) * rand () / (RAND_MAX + 1.0f) + road_min_x_;
		float y_val = (road_max_y_ - road_min_y_) * rand () / (RAND_MAX + 1.0f) + road_min_y_;
		float t_val = (max_theta_ - min_theta_) * rand () / (RAND_MAX + 1.0f) + min_theta_;

		vehicle_configs_.col(i_idx) = Eigen::Vector3f(x_val,y_val,t_val);			
	}

	int x_idx = 0;
	for (int e_idx=config_count-config_x_count_; e_idx < config_count; e_idx++)
	{
		float x_val = road_min_x_ + x_idx*x_resolution;;
		float y_val = road_max_y_;
		float t_val = 90;
		x_idx++;

		vehicle_configs_.col(e_idx) = Eigen::Vector3f(x_val,y_val,t_val);	
	}
	//	std::cout <<"rows:" << vehicle_configs_.rows() << "\t cols:" << vehicle_configs_.cols() << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRM<PointT>::validateConfigurations ()
{
	//	Eigen::Vector3f config(1,1,90);
	//	Eigen::MatrixXf vehicle_state = getVehicleState(config);
	//
	//	std::cout << "vehicle state: " << vehicle_state << std::endl;
	//	float state_min_x = vehicle_state.row(0).leftCols(4).minCoeff();
	//	float state_min_y = vehicle_state.row(1).leftCols(4).minCoeff();
	//	std::cout << "state min x: " << state_min_x << "\t min y:" << state_min_y << std::endl;

	int config_count = config_x_count_ * config_y_count_ * config_theta_count_;
	config_collision_status_.resize(config_count,-1);
	int valid_count=0;
	int invalid_count=0;
	int counter=0;
	for (int c_idx=0; c_idx < config_count; c_idx++)
	{
		Eigen::MatrixXf vehicle_state = getVehicleState(vehicle_configs_.col(c_idx));
		//		std::cout << "vehicle state: " << vehicle_state << std::endl;
		//		float state_min_x = vehicle_state.row(1).leftCols(4).minCoeff();
		bool config_valid_flag = collisionChecker(vehicle_state);
		if(!config_valid_flag)
		{
			config_collision_status_[c_idx] = 1;
			valid_count++;
		} else
		{
			config_collision_status_[c_idx] = 0;
			invalid_count++;
		}
		counter++;
	}

	//	default_dem_cells_ = tmp_dem_cells.leftCols(cell_counter);
	std::cout <<"valid:" << valid_count << "\t invalid:" << invalid_count <<"\t counter:" << counter << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRM<PointT>::generatePRMGraph ()
{
	getConfigCloud();
	int config_count = config_x_count_ * config_y_count_ * config_theta_count_;
	std::vector< std::pair<int,int> > prm_graph;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_prm (new pcl::search::KdTree<pcl::PointXYZ>);
	tree_prm->setInputCloud (cloud_prm_);

	for (int c_idx=0; c_idx < cloud_prm_->points.size(); c_idx++)
	{
		//		std::cout <<"finding connectivity of node: \t" << c_idx << std::endl;
		if(config_collision_status_[c_idx] != 1)
		{
			continue;
		} else
		{
			/*
			 * Get neighbors configuration and see if they are reachable from current configuration.
			 */
			std::vector<int> neighbor_idx = getTempNeighbors(c_idx);

			//			std::vector<int> neighbor_idx;
			//			std::vector<float> neighbor_dist;
			//			tree_prm ->radiusSearch (cloud_prm_->points[c_idx], config_search_radius_, neighbor_idx, neighbor_dist);
			//			std::cout <<"neighboring configuration count: \t"<<neighbor_idx.size() << std::endl;
			for (int n_idx=0; n_idx < neighbor_idx.size(); n_idx++)
			{
				if(neighbor_idx[n_idx] > config_count)
				{
					continue;
				}
				Eigen::Vector3f c_config(vehicle_configs_.col(c_idx));
				Eigen::Vector3f n_config(vehicle_configs_.col(neighbor_idx[n_idx]));
				//				std::cout <<"config: \t"<<c_config.transpose() <<" \t n_config:"<<n_config.transpose() << std::endl;//<<"\t dist:"<<neighbor_dist[n_idx]<< std::endl;
				//					continue;

				if(config_collision_status_[neighbor_idx[n_idx]] != 1 )
				{
					continue;
				} else 
				{
					/*
					 * if neighbor is valid then sub-sample and find a valid path
					 */
					bool is_neighbor_reachable = findConfigsConnectivity(c_idx, neighbor_idx[n_idx]);
					if(is_neighbor_reachable)
					{
						prm_graph.push_back(std::make_pair(c_idx,neighbor_idx[n_idx]));
					}
				}
			}
		}
	}
	prm_graph_ = prm_graph;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::PRM<PointT>::collisionChecker (Eigen::MatrixXf vehicle_state)
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
template <typename PointT> bool
pcl::PRM<PointT>::checkDemUnderVehicle (int d_idx, Eigen::MatrixXf vehicle_body_state)
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
pcl::PRM<PointT>::tyreClusterCollisionProcessing (int d_idx)
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
pcl::PRM<PointT>::bodyClusterCollisionProcessing (int d_idx)
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
pcl::PRM<PointT>::getVehicleState (Eigen::Vector3f v_config)
{
	Eigen::MatrixXf vehicle_state_origin(3,8);
	vehicle_state_origin << -vehicle_width_/2,vehicle_width_/2,vehicle_width_/2,-vehicle_width_/2,-front_track_/2,front_track_/2, front_track_/2, -front_track_/2,
			-vehicle_length_/2,-vehicle_length_/2,vehicle_length_/2,vehicle_length_/2,-wheelbase_/2,-wheelbase_/2,wheelbase_/2,wheelbase_/2,
			0,0,0,0,0,0,0,0;
	Eigen::Vector3f normal_xy_plane (0.0,0.0,1.0);
	//	std::cout << "vehicle default state: " << vehicle_state_origin << std::endl;
	/*
	 *  Find the vehicle position using its configuration.
	 */
	Eigen::MatrixXf position_offset(3,8);
	position_offset << v_config.head(2).replicate(1,8) , Eigen::MatrixXf::Zero(1,8);
	//	std::cout << "config:" << v_config << "\t position_offset:" << position_offset << std::endl;
	Eigen::AngleAxis<float> transform((PI_*cos(90-v_config(2)))/180,normal_xy_plane);
	Eigen::Matrix3f tranform_matrix;
	tranform_matrix = transform.matrix();
	Eigen::MatrixXf vehicle_state_rotated(3,8);
	vehicle_state_rotated = tranform_matrix*vehicle_state_origin;

	Eigen::MatrixXf vehicle_state(3,8);
	vehicle_state = vehicle_state_rotated + position_offset;
	return vehicle_state;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//template <typename PointT> void
//pcl::PRM<PointT>::getDEMCellsCovered ()
//{
//	int dem_cells_count = cloud_dem_->points.size();
//	Eigen::MatrixXf tmp_dem_cells = Eigen::MatrixXf(3,dem_cells_count);
//	int cell_counter = 0;
//	for(int p_idx=0; p_idx < dem_cells_count; p_idx++)
//	{
//		float x_val = cloud_dem_->points[p_idx].x;
//		float y_val = cloud_dem_->points[p_idx].y;
//		if(x_val >= -vehicle_width_/2 && x_val <= vehicle_width_/2 && y_val >= -vehicle_length_/2 && y_val <= vehicle_width_/2)
//		{
//			tmp_dem_cells.col(p_idx) << Eigen::Vector3f(x_val,y_val,1);
//			cell_counter++;
//		}		
//	}
//	default_dem_cells_ = tmp_dem_cells.leftCols(cell_counter);
//	std::cout <<"rows:" << default_dem_cells_.rows() << "\t cols:" << default_dem_cells_.cols() <<"\t counter:" << cell_counter << std::endl;
//}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PRM<PointT>::computePRM ()
{
	bool computation_is_possible = initCompute ();
	if ( !computation_is_possible )
	{
		deinitCompute ();
		return;
	}

	bool is_ready = preProcessingPRM();
	if(!is_ready)
	{
		std::cout << "Error, issue in pre-processing for PRM." << std::endl;
		return;
	}

	std::cout << "Generating configurations." << std::endl;
	if(is_random_config_)
	{
		generateRandomConfigurations();
	}else
	{
		generateConfigurations();
	}
	std::cout << "Validating configurations." << std::endl;
	validateConfigurations();
	//std::cout << "Generating PRM Graph." << std::endl;
	//generatePRMGraph();

	//std::cout << "PRM graph has edge:\t" << prm_graph_.size()<< std::endl;
	deinitCompute ();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::PRM<PointT>::preProcessingPRM ()
{
	std::cout << "Pre-processing for PRM." << std::endl;
	// if user forgot to pass point cloud or if it is empty
	//	if ( input_->points.size () == 0 )
	//		return (false);

	//	// check whether input cloud is already projected
	//	if(!is_projected_)
	//	{
	//		projectPointCloud();
	//	} 

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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> pcl::PointCloud<pcl::PointXYZRGB>::Ptr
pcl::PRM<PointT>::getPRMVisibilityCloud ()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
	colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();

	for (int i=0; i < config_collision_status_.size(); i++)
	{
		pcl::PointXYZRGB point;
		point.x = vehicle_configs_.col(i)[0];
		point.y = vehicle_configs_.col(i)[1];
		point.z = 0;
		point.r = 0;
		point.g = 0;
		point.b = 0;

		if (config_collision_status_[i] == -1)
		{
			point.b = 255;
		} else if (config_collision_status_[i] == 0)
		{
			point.r = 255;
		} else
		{
			point.g = 255;
		}
		colored_cloud->points.push_back (point);
	}

	return (colored_cloud);
}

#define PCL_INSTANTIATE_PRM(T) template class pcl::PRM<T>;

#endif /* PRM_HPP_ */
