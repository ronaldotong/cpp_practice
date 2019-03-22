#include <stdlib.h>
#include "graph.hpp"
#include "sort.hpp"

auto dump_path = [](std::stack<uint32_t>& path) {
    std::cout << "[" << path.top() << "]";
    path.pop();
    while (!path.empty()) {
        std::cout << " --> [" << path.top() << "]";
        path.pop();
    }
    std::cout << std::endl;
};

auto dump_ele = [](const std::vector<int>& vec) {
    std::cout << "============================================" << std::endl;
    std::cout << "Now Dump Sort Element" << std::endl;
    std::cout << "============================================" << std::endl;
    for (auto ele : vec) {
        std::cout << " " << ele;
    }
    std::cout << std::endl;
};

int main(int, char**) {
    // Test Merge
    std::vector<int> arr1{3, 4, 7, 9};
    std::vector<int> arr2{2, 5, 6, 7};
    std::vector<int> result(arr1.size() + arr2.size());
    auto out_iterator =
        sort::merge(arr1.begin(), arr1.end(), arr2.begin(), arr2.end(), result.begin());
    dump_ele(result);

    // Test Merge Sort
    std::vector<int> test1{2, 1, 5, 3, 7, 8, 9, 44, 3};
    auto result_vec = sort::merge_sort(test1);
    dump_ele(result_vec);

    // Constract Node
    Graph<SimpleGraphNode, GraphEdge> graph;
    graph.addNode(std::make_shared<SimpleGraphNode>(0, "0"));
    graph.addNode(std::make_shared<SimpleGraphNode>(1, "1"));
    graph.addNode(std::make_shared<SimpleGraphNode>(2, "2"));
    graph.addNode(std::make_shared<SimpleGraphNode>(3, "3"));
    graph.addNode(std::make_shared<SimpleGraphNode>(4, "4"));
    graph.addNode(std::make_shared<SimpleGraphNode>(5, "5"));
    graph.addNode(std::make_shared<SimpleGraphNode>(6, "6"));
    graph.addNode(std::make_shared<SimpleGraphNode>(7, "7"));
    graph.addNode(std::make_shared<SimpleGraphNode>(8, "8"));

    // Construct Graph
    // Node: 0 Path: 0-->[1] ~~ [7]
    // Node: 1 Path: 1-->[0] ~~ [2] ~~ [7]
    // Node: 2 Path: 2-->[1] ~~ [3] ~~ [8] ~~ [5]
    // Node: 3 Path: 3-->[2] ~~ [4] ~~ [5]
    // Node: 4 Path: 4-->[3] ~~ [5] ~~
    // Node: 5 Path: 5-->[2] ~~ [3] ~~ [4] ~~ [6]
    // Node: 6 Path: 6-->[5] ~~ [7] ~~ [8]
    // Node: 7 Path: 7-->[0] ~~ [1] ~~ [6] ~~ [8]
    // Node: 8 Path: 8-->[2] ~~ [6] ~~ [7]

    graph.addEdge(0, 1, 4);
    graph.addEdge(0, 7, 8);
    graph.addEdge(1, 2, 8);
    graph.addEdge(1, 7, 11);
    graph.addEdge(2, 3, 7);
    graph.addEdge(2, 8, 2);
    graph.addEdge(2, 5, 4);
    graph.addEdge(3, 4, 9);
    graph.addEdge(3, 5, 14);
    graph.addEdge(4, 5, 10);
    graph.addEdge(5, 6, 2);
    graph.addEdge(6, 7, 1);
    graph.addEdge(6, 8, 6);
    graph.addEdge(7, 8, 7);

    std::cout << "============================================" << std::endl;
    std::cout << "Now Dump Graph Info" << std::endl;
    std::cout << "============================================" << std::endl;
    graph.dumpGraphInfo();

    std::cout << "============================================" << std::endl;
    std::cout << "Now Dump Shortest Distance from Node 0" << std::endl;
    std::cout << "============================================" << std::endl;
    graph.shortestPath(0);

    std::cout << "============================================" << std::endl;
    std::cout << "Now Dump DFS Search From Node 0 to Node 4" << std::endl;
    std::cout << "============================================" << std::endl;
    std::stack<uint32_t> path_dfs;
    bool ret = graph.searchDFS(0, 4, path_dfs);
    if (ret) {
        dump_path(path_dfs);
    }

    std::cout << "============================================" << std::endl;
    std::cout << "Now Dump BFS Search From Node 0 to Node 4" << std::endl;
    std::cout << "============================================" << std::endl;
    ret = graph.searchBFS(0, 4, path_dfs);
    if (ret) {
        dump_path(path_dfs);
    }
}
