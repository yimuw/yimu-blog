#include <chrono>

#include "third_party/nanoflann.hpp"
#include "third_party/utils.h"
#include "3dtree.h"


std::vector<geometry::Point3d> convert_to_my_point(const PointCloud<float> &pc)
{
    std::vector<geometry::Point3d> result;
    for(const auto &p : pc.pts)
    {
        result.emplace_back(p.x, p.y, p.z);
    }
    return result;
}

struct Profiler
{
    using time_point = std::chrono::steady_clock::time_point;

    Profiler(const std::string &info)
        : info_(info)
    {
        begin_ = std::chrono::steady_clock::now();
    }
    
    ~Profiler()
    {
        time_point end = std::chrono::steady_clock::now();
        auto elapsed_msec = std::chrono::duration_cast<std::chrono::microseconds>(end - begin_).count();
        std::cout << "profiler info: " << info_ << std::endl;
        std::cout << "time elapsed milliseconds: " << elapsed_msec << std::endl;
    }

    std::string info_;
    time_point begin_;
};

int main()
{
    PointCloud<float> cloud;
	// Generate points:
	generateRandomPointCloud(cloud, 1000000);

    std::vector<geometry::Point3d> my_points = convert_to_my_point(cloud);

    geometry::ThreeDimTree tree3d;
    {
        Profiler p("build_tree_recursive");
        tree3d.build_tree_recursive(my_points);
    }
    std::cout << "build_tree_recursive done" << std::endl;

    geometry::Point3d target = my_points.front();
    target.x += 0.5;

    
    {
        Profiler p("nearst_neighbor_search_recursive");
        auto nn_result = tree3d.nearst_neighbor_search_recursive(target);
        // std::cout << "nn_result: " << nn_result.success << std::endl;
        // std::cout << "target:" << target << std::endl;
        // std::cout << "nn_result.nearest_point:" << nn_result.nearest_point << std::endl;
        std::cout << "nn_result.distance:" << nn_result.distance_square << std::endl;
    }




    // construct a kd-tree index:
	typedef nanoflann::KDTreeSingleIndexAdaptor<
		nanoflann::L2_Simple_Adaptor<float, PointCloud<float> > ,
		PointCloud<float>,
		3 /* dim */
		> my_kd_tree_t;


	my_kd_tree_t   index(3 /*dim*/, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(1 /* max leaf */) );
	{
        Profiler p("buildIndex");
        index.buildIndex();
    }

	{
		// do a knn search
        float query_pt[3] = {target.x, target.y, target.z};
		const size_t num_results = 1;
		size_t ret_index;
		float out_dist_sqr;
		nanoflann::KNNResultSet<float> resultSet(num_results);
		resultSet.init(&ret_index, &out_dist_sqr );

        Profiler p("findNeighbors");
		index.findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));

		std::cout << "knnSearch(nn="<<num_results<<"): \n";
		std::cout << "ret_index=" << ret_index << " out_dist_sqr=" << out_dist_sqr << std::endl;
	}

    return 0;
}