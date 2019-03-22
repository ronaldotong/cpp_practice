#include <algorithm>
#include <map>
#include <stack>
#include <queue>
#include <memory>
#include <string.h>
#include <iostream>
namespace sort {
template <typename InputIterator1, typename InputIterator2, typename OutputIterator>
OutputIterator merge(InputIterator1 itr_left,
    InputIterator1 itr_left_end,
    InputIterator2 itr_right,
    InputIterator2 itr_right_end,
    OutputIterator itr_result) {
    while (itr_left != itr_left_end && itr_right != itr_right_end) {
        if (*itr_left < *itr_right) {
            *itr_result = *itr_left;
            ++itr_left;
        } else {
            *itr_result = *itr_right;
            ++itr_right;
        }
        ++itr_result;
    }
    return std::copy(itr_right, itr_right_end, std::copy(itr_left, itr_left_end, itr_result));    
}

template<typename Type>
Type merge_sort(const Type& array) {
    if (array.size() <= 1)
        return array;
    using Itr = typename Type::iterator;
    using ValueType = typename Type::value_type;
    using Pair = std::pair<Itr, Itr>;
    Type result(array.size());
    Type origin_copy(array);
    Itr begin = origin_copy.begin();
    std::queue<Pair> q;
    while(begin != origin_copy.end()){
        auto next = begin + 1;
        q.push(std::make_pair(begin, next));
        ++begin;
    }
    Itr result_itr = result.begin();
    while (q.size() > 1) {
        auto left = q.front();
        q.pop();
        if (left.second == origin_copy.end()){
            q.push(left);
            result_itr = result.begin();
            continue;
        }
        auto right = q.front();
        q.pop();
        result_itr = sort::merge(left.first, left.second, right.first, right.second, result_itr);
        size_t element_size = right.second - left.first;
        memcpy(&*(left.first), &*(result_itr - element_size), sizeof(ValueType) * element_size);
        q.push(std::make_pair(left.first, right.second));
    }
    return result;
}
}