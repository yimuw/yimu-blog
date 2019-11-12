#pragma once

#include <algorithm>
#include <assert.h>
#include <vector>
#include <iostream>


namespace geometry
{

constexpr float MAX_FLOAT = std::numeric_limits<float>::max();
constexpr float MIN_FLOAT = std::numeric_limits<float>::min();

// Yet another point3d
struct Point3d
{
    Point3d() = default;

    Point3d(float x, float y, float z)
        : x(x), y(y), z(z)
    {
    }

    inline const float& operator [](const int i) const
    {
        if(i == 0)
        {
            return x;
        }
        else if(i == 1)
        {
            return y;
        }
        else if(i == 2)
        {
            return z;
        }
        else
        {
            assert(false);
        }
    }

    // TODO: const cast
    inline float& operator [](const int i)
    {
        if(i == 0)
        {
            return x;
        }
        else if(i == 1)
        {
            return y;
        }
        else if(i == 2)
        {
            return z;
        }
        else
        {
            assert(false);
        }
    }

    float x = {0.};
    float y = {0.};
    float z = {0.};
};

inline std::ostream& operator<<(std::ostream& os, const Point3d& p)
{
    os << p.x << "," << p.y << "," << p.z;
    return os;
}

inline float distance_square(const Point3d &p1, const Point3d &p2)
{
    const float d1 = p1.x - p2.x;
    const float d2 = p1.y - p2.y;
    const float d3 = p1.z - p2.z;
    return d1 * d1 + d2 * d2 + d3 * d3;
}

// Utils & helpers for 3d tree
namespace utils
{
struct TreeIndex
{
    TreeIndex() = default;

    TreeIndex(int start_idx, int end_idx, int depth)
        : start_index(start_idx), end_index(end_idx), tree_depth(depth)
    {
        // mid point is pivot 
        pivot_index = (start_index + end_index) / 2;
    }

    int pivot_index = {0};
    // [start_index, end_index), because of std::sort
    int start_index = {0};
    int end_index = {0};
    int tree_depth = {0};
};

inline void pivot_tree(const TreeIndex &current_tree,
                TreeIndex &left_tree,
                TreeIndex &right_tree)
{
    // [start, pivot)
    left_tree = {current_tree.start_index, current_tree.pivot_index, 
        current_tree.tree_depth + 1};
    // [pivot + 1, end)
    right_tree = {current_tree.pivot_index + 1, current_tree.end_index, 
        current_tree.tree_depth + 1};
}

struct Hyperrectangle3d
{
    float max_for_dim_i(const int i) const
    {
        return maxs.at(i);
    }

    float& max_for_dim_i(const int i)
    {
        return maxs.at(i);
    }

    float min_for_dim_i(const int i) const
    {
        return mins.at(i);
    }

    float& min_for_dim_i(const int i)
    {
        return mins.at(i);
    }

private:
    std::vector<float> maxs = std::vector<float>(3, MIN_FLOAT);
    std::vector<float> mins = std::vector<float>(3, MAX_FLOAT);
};

    Hyperrectangle3d compute_bounding_box(const std::vector<Point3d> &points)
    {
        Hyperrectangle3d result;
        for(const auto &p : points)
        {
            for(size_t point_dim = 0; point_dim < 3; ++point_dim)
            {
                result.max_for_dim_i(point_dim) 
                    = std::max(result.max_for_dim_i(point_dim), p[point_dim]);
                result.min_for_dim_i(point_dim) 
                    = std::min(result.min_for_dim_i(point_dim), p[point_dim]);
            }
        }
        return result;
    }

inline void pivot_bounding_box(const Hyperrectangle3d &current_bbox,
                        const int dim_to_pivot,
                        const float pivot_value,
                        Hyperrectangle3d &left_bbox,
                        Hyperrectangle3d &right_bbox)
{
    left_bbox = current_bbox;
    left_bbox.max_for_dim_i(dim_to_pivot) = pivot_value;

    right_bbox = current_bbox;
    right_bbox.min_for_dim_i(dim_to_pivot) = pivot_value;
}

float distance_square_bbox_to_point(const Hyperrectangle3d &bbox,
                                    const Point3d &target)
{
    Point3d closest_point_in_bbox;
    for(int dim = 0; dim < 3; ++dim)
    {
        const float target_val_i = target[dim];
        if(target_val_i < bbox.min_for_dim_i(dim))
        {
            closest_point_in_bbox[dim] = bbox.min_for_dim_i(dim);
        }
        else if(target_val_i > bbox.max_for_dim_i(dim))
        {
            closest_point_in_bbox[dim] = bbox.max_for_dim_i(dim);
        }
        else
        {
            closest_point_in_bbox[dim] = target_val_i;
        }
    }
    return distance_square(closest_point_in_bbox, target);
}

}

class ThreeDimTree
{
public:
//////////////////////////////////////////////////////////////////////////////
// Build tree
//////////////////////////////////////////////////////////////////////////////

    void build_tree_recursive(const std::vector<Point3d> &points)
    {
        assert(points.size() > 0);

        kdtree_data_ = points;
        tree_bounding_box_ = utils::compute_bounding_box(kdtree_data_);
        tree_index_ = {0, static_cast<int>(kdtree_data_.size()), 0};

        // open interval
        build_tree_recursive(tree_index_);
    }

    void build_tree_recursive(const utils::TreeIndex &tree_index)
    {
        // base case
        if(tree_index.end_index - tree_index.start_index <= 1)
        {
            return;
        }
        
        const int dim = tree_index.tree_depth % 3;

        std::sort(kdtree_data_.begin() + tree_index.start_index, 
                  kdtree_data_.begin() + tree_index.end_index,
                  [dim](const Point3d &p1, const Point3d &p2)
                  {
                      return p1[dim] < p2[dim];
                  });

        utils::TreeIndex left_tree, right_tree;
        pivot_tree(tree_index, left_tree, right_tree);

        build_tree_recursive(left_tree);
        build_tree_recursive(right_tree);

        if(false)
        {
            const float pivot_value = kdtree_data_.at(tree_index.pivot_index)[dim];
            for(int i = tree_index.start_index; i < tree_index.pivot_index; ++i)
            {
                assert(kdtree_data_[i][dim] <= pivot_value);
            }
            for(int i = tree_index.pivot_index; i < tree_index.end_index; ++i)
            {
                assert(kdtree_data_[i][dim] >= pivot_value);
            }
        }
    }

//////////////////////////////////////////////////////////////////////////////
// Nearest neighbor
//////////////////////////////////////////////////////////////////////////////

    struct NearestNeighborSearchStruct
    {
        NearestNeighborSearchStruct(const utils::TreeIndex &tree,
                                    const utils::Hyperrectangle3d &bbox,
                                    const Point3d &target)
            : current_tree(tree), current_tree_bounding_box(bbox), target(target)
        {
        }

        void check(const std::vector<Point3d> &points) const
        {
            for(int i = current_tree.start_index; i < current_tree.end_index; ++i)
            {
                for(int dim = 0; dim < 3; ++dim)
                {
                    float v = points.at(i)[dim];
                    assert(v >= current_tree_bounding_box.min_for_dim_i(dim));
                    assert(v <= current_tree_bounding_box.max_for_dim_i(dim));
                }
            }
        }

        utils::TreeIndex current_tree;
        utils::Hyperrectangle3d current_tree_bounding_box;
        Point3d target;
    };

    struct NearestNeightResult
    {
        bool success = false;
        Point3d nearest_point;
        float distance_square = std::numeric_limits<float>::max();
    };

    NearestNeightResult nearst_neighbor_search_recursive(const Point3d &target)
    {
        NearestNeighborSearchStruct nn_struct(tree_index_, tree_bounding_box_, target);
        Point3d nearst_point;
        float max_distance_square = std::numeric_limits<float>::max();
        nearst_neighbor_search_recursive(nn_struct, nearst_point, max_distance_square);

        NearestNeightResult result;
        if(max_distance_square != MAX_FLOAT)
        {
            result.success = true;
            result.nearest_point = nearst_point;
            result.distance_square = max_distance_square;
        }
        else
        {
            result.success = false;
        }
        return result;
    }

    void nearst_neighbor_search_recursive(const NearestNeighborSearchStruct &nn_struct,
                                          Point3d &nearst_point,
                                          float &max_distance_square)
    {
        const auto &current_tree = nn_struct.current_tree;
        const auto &target = nn_struct.target;
            
        // nn_struct.check(kdtree_data_);

        // base case
        // leaf node, NOTE: [start_index, end_index)
        if(current_tree.start_index + 1 == current_tree.end_index)
        {
            //std::cout << "rr" << std::endl;

            const Point3d &leaf = kdtree_data_.at(current_tree.start_index);
            const float dist_sqr 
                = distance_square(target, leaf);
            //std::cout << "dist_sqr: " << dist_sqr << std::endl;
            //std::cout << "max_distiance_square: " << max_distance_square << std::endl;
            if(dist_sqr < max_distance_square)
            {
                max_distance_square = dist_sqr;
                nearst_point = leaf;
            }
            return;
        }
        // empty
        else if(current_tree.start_index == current_tree.end_index)
        {
            std::cout << "empty" << std::endl;
            return;
        }

        const int pivot_idx = current_tree.pivot_index;
        const int dim_to_check = current_tree.tree_depth % 3;
        const auto &pivot_pt = kdtree_data_.at(pivot_idx);
        const float pivot_value = pivot_pt[dim_to_check]; 

        // Since pivot is not included in the partition
        const float pivot_distance = distance_square(pivot_pt, target);
        if(pivot_distance < max_distance_square)
        {
            max_distance_square = pivot_distance;
            nearst_point = pivot_pt;
        }

        // Cut the space into half
        // TODO: I think make a copy is fine for DFS. But it is better to profile it.
        NearestNeighborSearchStruct left_nn_struct = nn_struct;
        NearestNeighborSearchStruct right_nn_struct = nn_struct;
        pivot_tree(current_tree, left_nn_struct.current_tree, right_nn_struct.current_tree);;
        pivot_bounding_box(nn_struct.current_tree_bounding_box, dim_to_check, pivot_value,
            left_nn_struct.current_tree_bounding_box, right_nn_struct.current_tree_bounding_box);


        // Decide which half is more likely to success.
        const float diff = target[dim_to_check] - pivot_value;
        NearestNeighborSearchStruct &first_to_check = diff > 0 ? right_nn_struct : left_nn_struct;
        NearestNeighborSearchStruct &second_to_check = diff > 0 ? left_nn_struct : right_nn_struct;
        
        // Check the most likely space first
        nearst_neighbor_search_recursive(first_to_check, nearst_point, max_distance_square);

        // nearst_neighbor_search_recursive(second_to_check, nearst_point, max_distance_square);

        // Pruning
        const float dist_lower_bound_second_half 
            = distance_square_bbox_to_point(second_to_check.current_tree_bounding_box, target);
        if(dist_lower_bound_second_half < max_distance_square)
        {
            nearst_neighbor_search_recursive(second_to_check, nearst_point, max_distance_square);
        }
        else
        {
            // std::cout << "pruned, dist_lower_bound:" << dist_lower_bound_second_half << std::endl;
        }
        
    }

private:
    std::vector<Point3d> kdtree_data_;
    utils::Hyperrectangle3d tree_bounding_box_;
    utils::TreeIndex tree_index_;
};

}