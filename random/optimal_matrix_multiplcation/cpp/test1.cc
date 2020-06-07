#include <iostream>
#include <array>


// A binary tree of height N
template <std::size_t N>
struct binary_tree
{
    constexpr binary_tree() {};

    int value = N;
    bool is_left = false;

    binary_tree<N-1> left;
    binary_tree<N-1> right;
    constexpr static const std::size_t capacity = 2^N -1;
};

template <>
struct binary_tree<0>
{
    constexpr binary_tree(){};

    std::size_t size{0};
    constexpr static const bool nil = true;
};

////////////////////////////////////////////////

template<size_t N>
struct Traverse
{
    constexpr Traverse(binary_tree<N> tree)
    {
        traverse_inner(tree);
    }

    template<size_t K>
    constexpr void traverse_inner(binary_tree<K> tree) {
        a[count] = tree.value;
        count += 1;

        //traverse_inner<K-1>(tree.left);
        //traverse_inner<K-1>(tree.right);
        
    }

    // template<>
    // constexpr void traverse_inner<1>(binary_tree<1> tree) {
    //     a[count] = tree.value;
    //     count += 1;
    // }

    int count = 0;
    constexpr static const std::size_t capacity = 2^N -1;
    std::array<char, (N^2) - 1> a;
};


int main(int argc, char *argv[])
{
    constexpr auto tree = binary_tree<3>();
    static_assert(tree.value == 3);
    constexpr auto t = Traverse<3>(tree);
    return 0;
}