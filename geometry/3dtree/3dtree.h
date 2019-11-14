#pragma once

#include <algorithm>
#include <assert.h>
#include <vector>
#include <iostream>
#include <memory>
#include <array>
#include <stack> 


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
struct Hyperrectangle3d
{
    inline float max_for_dim_i(const int i) const
    {
        return maxs[i];
    }

    inline float& max_for_dim_i(const int i)
    {
        return maxs[i];
    }

    inline float min_for_dim_i(const int i) const
    {
        return mins[i];
    }

    inline float& min_for_dim_i(const int i)
    {
        return mins[i];
    }

private:
    std::array<float, 3> maxs = {0., 0., 0.};
    std::array<float, 3> mins = {0., 0., 0.};
};

inline Hyperrectangle3d compute_bounding_box(const std::vector<Point3d> &points)
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

inline float distance_square_bbox_to_point(const Hyperrectangle3d &bbox,
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
    struct TreeNode
    {
        size_t pivot_index = 0;
        size_t dim = 0;
        float pivot_value = -1;

        inline bool is_leaf() const
        {
            return left_child == nullptr && right_child == nullptr;
        }

        TreeNode* left_child = nullptr;
        TreeNode* right_child = nullptr;
    };

    // raw pointer, 20% performance gain
    using TreeNodePtr = TreeNode*;

    ~ThreeDimTree()
    {
        // TODO: delete tree
    };
//////////////////////////////////////////////////////////////////////////////
// Build tree
//////////////////////////////////////////////////////////////////////////////

    void build_tree_recursive(const std::vector<Point3d> &points)
    {
        assert(points.size() > 0);

        kdtree_data_ = points;
        tree_bounding_box_ = utils::compute_bounding_box(kdtree_data_);

        // open interval
        kdtree_root_ = new TreeNode;
        build_tree_recursive(kdtree_root_, 0, kdtree_data_.size(), tree_bounding_box_);
    }

    int select_dim(const utils::Hyperrectangle3d &bbox)
    {
        float max_range = MIN_FLOAT;
        int widest_dim = -1;
        for(int dim = 0; dim < 3; ++dim)
        {
            float range = bbox.max_for_dim_i(dim) - bbox.min_for_dim_i(dim);
            // assert(range >= 0);
            if(range > max_range)
            {
                max_range = range;
                widest_dim = dim;
            }
        }
        return widest_dim;
    }

    void build_tree_recursive(TreeNodePtr tree_node_ptr, 
                              int left_index, int right_index, const utils::Hyperrectangle3d &bbox)
    {
        // assert(tree_node_ptr != nullptr);
        // assert(left_index < right_index);
        // base case
        if(right_index - left_index <= 1)
        {
            // std::cout << "leaf, index: " << left_index << std::endl;
            tree_node_ptr->pivot_index = left_index;
            return;
        }
        
        const int dim = select_dim(bbox);
        // const int dim = tree_node_ptr->depth % 3;

        std::sort(kdtree_data_.begin() + left_index, 
                  kdtree_data_.begin() + right_index,
                  [dim](const Point3d &p1, const Point3d &p2)
                  {
                      return p1[dim] < p2[dim];
                  });
        const int pivot_index = (left_index + right_index) / 2;
        const float pivot_value = kdtree_data_.at(pivot_index)[dim];

        utils::Hyperrectangle3d left_bbox = bbox, right_bbox = bbox;
        left_bbox.max_for_dim_i(dim) = pivot_value;
        right_bbox.min_for_dim_i(dim) = pivot_value;

        tree_node_ptr->dim = dim;
        tree_node_ptr->pivot_value = pivot_value;
        tree_node_ptr->pivot_index = pivot_index;

        // std::cout << "subtree size: " << pivot_index - left_index << ", " << right_index - pivot_index << std::endl;

        // TODO: Can be parallelize since left and right doesn't intersect.
        if(pivot_index != left_index)
        {
            tree_node_ptr->left_child = new TreeNode;
            build_tree_recursive(tree_node_ptr->left_child, left_index, pivot_index, left_bbox);
        }
        
        if(pivot_index + 1 != right_index)
        {
            tree_node_ptr->right_child = new TreeNode;
            build_tree_recursive(tree_node_ptr->right_child, pivot_index + 1, right_index, right_bbox);
        }
 
    }

//////////////////////////////////////////////////////////////////////////////
// Nearest neighbor
//////////////////////////////////////////////////////////////////////////////
    using distance3d = std::array<float, 3>;

    struct NearestNeightResult
    {
        bool success = false;
        Point3d nearest_point;
        float distance_square = std::numeric_limits<float>::max();
    };

    void init_rect_distance(const Point3d &target, distance3d &dist, float &max_distance_square)
    {
        for(int dim = 0; dim < 3; ++dim)
        {
            const float bb_min = tree_bounding_box_.min_for_dim_i(dim);
            const float bb_max = tree_bounding_box_.max_for_dim_i(dim);
            const float target_val = target[dim];
            if(target_val < bb_min)
            {
                dist[dim] = (target_val - bb_min) * (target_val - bb_min);
            }
            else if(target_val > bb_max)
            {
                dist[dim] = (target_val - bb_max) * (target_val - bb_max);
            }
            else
            {
                dist[dim] = 0;
            }
        }

        max_distance_square = dist[0] + dist[1] + dist[2];
    }

    NearestNeightResult nearst_neighbor_search_recursive(const Point3d &target)
    {
        Point3d nearst_point;
        float max_distance_square = std::numeric_limits<float>::max();

        distance3d rect_dist_sqr_sep;
        float rect_min_dist_sqr;

        init_rect_distance(target, rect_dist_sqr_sep, rect_min_dist_sqr);
        nearst_neighbor_search_recursive(kdtree_root_, target, rect_min_dist_sqr, 
            rect_dist_sqr_sep, nearst_point, max_distance_square);

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
public:
    int nearst_neighbor_search_recursive_called = 0;

    const float search_epsilon = 1e-8;

    void check_update_distance(const Point3d &cur_pt,
                               const Point3d &target,
                               Point3d &nearst_point,
                               float &max_distance_square)
    {
        const float dist_sqr = distance_square(cur_pt, target);

        // std::cout << "=leaf idx:" << tree_node_ptr->leaf.left_index << std::endl;
        // std::cout << "dist_sqr: " << dist_sqr << std::endl;
        // std::cout << "max_distiance_square: " << max_distance_square << std::endl;
        if(dist_sqr < max_distance_square)
        {
            max_distance_square = dist_sqr;
            nearst_point = cur_pt;
        }
    }

    void nearst_neighbor_search_recursive(const TreeNodePtr tree_node_ptr,
                                          const Point3d &target,
                                          const float rect_min_dist_sqr,
                                          distance3d &rect_dist_sqr_sep,
                                          Point3d &nearst_point,
                                          float &max_distance_square)
    {
        nearst_neighbor_search_recursive_called++;
        // base case
        // leaf node, NOTE: [start_index, end_index)
        if(tree_node_ptr->is_leaf())
        {
            const Point3d &leaf = kdtree_data_[tree_node_ptr->pivot_index];
            check_update_distance(leaf, target, nearst_point, max_distance_square);
            return;
        }

        const Point3d &pivot_pt = kdtree_data_[tree_node_ptr->pivot_index];
        const float pivot_value = tree_node_ptr->pivot_value;
        const int dim = tree_node_ptr->dim;

        check_update_distance(pivot_pt, target, nearst_point, max_distance_square);
        // if(max_distance_square < 1e-3 * 1e-3)
        // {
        //     return;
        // }
        
        // Decide which half is more likely to success.
        const float diff = target[dim] - pivot_value;
        TreeNodePtr first_to_check, second_to_check;
        if(diff > 0)
        {
            first_to_check = tree_node_ptr->right_child;
            second_to_check = tree_node_ptr->left_child;
        }
        else
        {
            first_to_check = tree_node_ptr->left_child;
            second_to_check = tree_node_ptr->right_child;
        }
        
        // Check the most likely space first
        if(first_to_check != nullptr)
        {
            nearst_neighbor_search_recursive(first_to_check, target, rect_min_dist_sqr, rect_dist_sqr_sep, nearst_point, 
                max_distance_square);
        }
        // nearst_neighbor_search_recursive(second_to_check, nearst_point, max_distance_square);

        // Pruning
        if(second_to_check != nullptr)
        {
            const float prev_dist = rect_dist_sqr_sep[dim];
            rect_dist_sqr_sep[dim] = diff * diff;

            const float rect_min_dist_sqr_second = rect_min_dist_sqr - prev_dist + rect_dist_sqr_sep[dim];
            if(rect_min_dist_sqr_second < max_distance_square)
            {
                nearst_neighbor_search_recursive(second_to_check, target, rect_min_dist_sqr_second, rect_dist_sqr_sep,
                    nearst_point, max_distance_square);
            }
            rect_dist_sqr_sep[dim] = prev_dist;
        }
    }

    // 10% slower then recursive
    void nearst_neighbor_search_iterative_single_branch_comparison(const TreeNodePtr tree_root,
                                          const Point3d &target,
                                          const float rect_min_dist_sqr,
                                          distance3d &rect_dist_sqr_sep,
                                          Point3d &nearst_point,
                                          float &max_distance_square)
    {
        std::stack<TreeNodePtr> stack;
        stack.push(tree_root);

        while(not stack.empty())
        {
            nearst_neighbor_search_recursive_called++;

            TreeNodePtr tree_node_ptr = stack.top();
            stack.pop();

            if(tree_node_ptr->is_leaf())
            {
                const Point3d &leaf = kdtree_data_.at(tree_node_ptr->pivot_index);
                check_update_distance(leaf, target, nearst_point, max_distance_square);
                continue;
            }

            const Point3d &pivot_pt = kdtree_data_.at(tree_node_ptr->pivot_index);
            const float pivot_value = tree_node_ptr->pivot_value;
            const int dim = tree_node_ptr->dim;

            check_update_distance(pivot_pt, target, nearst_point, max_distance_square);

            // Decide which half is more likely to success.
            const float diff = target[dim] - pivot_value;
            const TreeNodePtr first_to_check = diff > 0 
                ? tree_node_ptr->right_child : tree_node_ptr->left_child;
            
            // Check the most likely space first
            if(first_to_check != nullptr)
            {
                stack.push(first_to_check);
            }
        }

    }

private:
    TreeNodePtr kdtree_root_;
    std::vector<Point3d> kdtree_data_;

    utils::Hyperrectangle3d tree_bounding_box_;
};

}
