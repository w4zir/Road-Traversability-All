#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <queue>
#include <list>

//#include "boost/tuple/tuple.hpp"

using namespace std;
using namespace pcl;

struct cluster_info
{
	float min_dist;
	float max_dist;
	int cluster_size;
};
/** \brief This function is used as a comparator for sorting. */
inline bool
comparePair (std::pair<float, int> i, std::pair<float, int> j)
{
	return (i.first < j.first);
}

int main(int argc, char** argv) {
	
	double values[] = {3.2,1.0,5.0,2.0,7.0,7.0,2.0,9.0,4.0};
	std::vector<int> point_labels_;
	int num_of_pts = 9;
	std::vector<double> dist;
	dist.resize(9,-1);

	for (int i=0; i<num_of_pts; i++)
	{
		std::cout <<"value at:" << i <<"\t of dist:" << dist[i] << std::endl;
		dist[i] = values[i];
		std::cout <<"value at:" << i <<"\t of dist after assignment:" << dist[i] << std::endl;
	}



	point_labels_.resize (9, -1);

	std::vector<std::vector< cluster_info > > point_ress;
	for (int j=0; j<5; j++)
	{
		std::vector< cluster_info > point_residual;
		int no = rand()%5;
		std::cout <<"no:"<< no << std::endl;
		for (int i=0; i<no; i++)
		{
			cluster_info cluster = {1.4,5.2,rand()%10};
//			cluster_info cluster(1.4,5.2,rand()%10);
//			cluster.min_dist = 1.4;
//			cluster.max_dist = 5.2;
//			cluster.cluster_size = rand()%10;
			point_residual.push_back(cluster);
			//point_residual.push_back(cluster_info info((float)(rand()%100),(float)(rand()%100),rand()%100));
			std::cout <<"point_residual size:"<< point_residual.size() << std::endl;
		}
		point_ress.push_back(point_residual);
		std::cout <<"size of std::vector< std::pair<float, int> >:" << point_ress[j].size() << std::endl;
	}
	std::cout <<"point_ress size:" << point_ress.size() << std::endl;
	for(int i=0; i<5; i++)
	{
		//		std::cout <<"point_res sizes:" << point_ress[i].size() << std::endl;
		for (int j=0; j<point_ress[i].size(); j++)			
		{
			//		std::cout <<"point_ress:" << point_ress[i] << std::endl;
			float first = point_ress[i][j].min_dist;
			float second = point_ress[i][j].max_dist;
			int third = point_ress[i][j].cluster_size;
//			std::tie(first, second, third) = point_ress[i][j];
			std::cout <<"first:" << first <<"\t second:" << second <<"\t thir:" << third << std::endl;
		}
	}
	//	std::pair<float, int> pair;
	//	point_residual.resize (num_of_pts, pair);

	//	for (int i_point = 0; i_point < num_of_pts; i_point++)
	//	{
	//		point_residual[i_point].first = dist[i_point];
	//		point_residual[i_point].second = i_point;
	//	}
	//	std::sort (point_residual.begin (), point_residual.end (), comparePair);
	//
	//	for(int i=0; i<num_of_pts; i++)
	//	{
	//		std::cout <<"first:" << point_residual[i].first <<"\t second:" << point_residual[i].second << std::endl;
	//	}
	return 0;
}
