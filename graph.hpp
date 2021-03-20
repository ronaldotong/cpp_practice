#pragma once
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <algorithm>
#include <array>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <queue>
#include <stack>
#include <vector>
const constexpr int32_t kInvalidSignedNumber = INT32_MAX;
const constexpr uint32_t kInvalidUnignedNumber = UINT32_MAX;

class GraphNode {
protected:
    uint32_t mIndex{kInvalidUnignedNumber};
    std::string mName{""};

public:
    GraphNode() : mIndex(kInvalidUnignedNumber) {}
    explicit GraphNode(uint32_t index) : mIndex(index) {}
    GraphNode(uint32_t index, const std::string& name) : mIndex(index), mName(name) {}
    virtual ~GraphNode() {}
    uint32_t getIndex() const { return mIndex; };
    void setIndex(uint32_t index) { mIndex = index; }
    const std::string& getName() { return mName; }
};

// You can define your own business node here.
// This is an example
using ExtraInfo = void*;
class SimpleGraphNode final : public GraphNode {
private:
    ExtraInfo mInfo{nullptr};  // reserve for extra usage
public:
    SimpleGraphNode() : GraphNode() {}
    SimpleGraphNode(uint32_t index, const std::string& name) : GraphNode(index, name) {}
    virtual ~SimpleGraphNode() {}
};

struct GraphEdge {
    GraphEdge(uint32_t fromIndex, uint32_t toIndex, int cost)
        : mFromIndex(fromIndex), mToIndex(toIndex), mCost(cost) {}
    ~GraphEdge() {}
    uint32_t mFromIndex;
    uint32_t mToIndex;
    int mCost;  // Use Template Would be Better.
};

template <typename NODE_TYPE, typename EDGE_TYPE>
class Graph {
public:
    using NodeType = NODE_TYPE;
    using EdgeType = EDGE_TYPE;
    using NodeVector = std::vector<std::shared_ptr<NodeType>>;
    using GraphEdgeList = std::list<std::shared_ptr<EdgeType>>;
    using GraphEdgeListVector = std::vector<GraphEdgeList>;
    enum VisitStatus : uint8_t { Unvisited, Visited };
    explicit Graph() {}
    ~Graph(){};
    int getNodeCount() const { return mNodes.size(); }

    const std::shared_ptr<NodeType>& getNode(int index) {
        if (index >= mNodes.size()) return nullptr;
        assert(index >= 0);
        return mNodes[index];
    }

    void addEdge(uint32_t srcId, uint32_t destId, int cost);
    void addEdge(const std::shared_ptr<GraphEdge>& edge);
    std::shared_ptr<EdgeType> getEdge(uint32_t from, uint32_t to);
    int addNode(const std::shared_ptr<NodeType>& node);
    void removeNode(int index);  // Please Finish the Definition
    void shortestPath(uint32_t srcId);
    void dumpGraphInfo();
    bool searchDFS(uint32_t srcId, uint32_t targetId, std::stack<uint32_t>& path);
    bool searchBFS(uint32_t srcId, uint32_t targetId, std::stack<uint32_t>& path);
    void getSearchPath(const std::vector<uint32_t>& route,
        uint32_t srcId,
        uint32_t targetId,
        std::stack<uint32_t>& path);
    int minTreeBuild();

private:
    std::shared_ptr<EdgeType>& findElementInPath(uint32_t srcId, uint32_t destId);
    void addUnidirectionalEdge(uint32_t srcId, uint32_t destId, int cost);
    NodeVector mNodes;
    GraphEdgeListVector mEdgeListVector;
    bool mDigraph{false};
};

template <typename NODE_TYPE, typename EDGE_TYPE>
void Graph<NODE_TYPE, EDGE_TYPE>::getSearchPath(const std::vector<uint32_t>& route,
    uint32_t srcId,
    uint32_t targetId,
    std::stack<uint32_t>& path) {
    assert(path.empty());
    uint32_t route_id = targetId;
    path.push(route_id);
    auto node_size = route.size();
    while (route_id != srcId) {
        assert(route_id != kInvalidUnignedNumber);
        assert(route_id < node_size);
        route_id = route[route_id];
        path.push(route_id);
    }
}

template <typename NODE_TYPE, typename EDGE_TYPE>
bool Graph<NODE_TYPE, EDGE_TYPE>::searchDFS(
    uint32_t srcId, uint32_t targetId, std::stack<uint32_t>& path) {
    std::stack<std::shared_ptr<EDGE_TYPE>> stack;
    // Push a dummy edge
    auto dummy = std::make_shared<EDGE_TYPE>(srcId, srcId, 0);
    assert(dummy != nullptr);
    stack.push(dummy);
    std::vector<uint32_t> route(mNodes.size(), kInvalidUnignedNumber);
    std::vector<VisitStatus> visit(mNodes.size(), VisitStatus::Unvisited);
    while (!stack.empty()) {
        auto& next_edge = stack.top();
        assert(next_edge != nullptr);
        stack.pop();
        uint32_t to = next_edge->mToIndex;
        uint32_t from = next_edge->mFromIndex;
        route[to] = from;
        visit[to] = VisitStatus::Visited;
        if (to == targetId) {
            getSearchPath(route, srcId, targetId, path);
            return true;
        }
        auto next_edge_list = mEdgeListVector[to];
        for (auto& edge : next_edge_list) {
            assert(edge != nullptr);
            if (visit[edge->mToIndex] == VisitStatus::Unvisited) stack.push(edge);
        }
    }
    return false;
}

template <typename NODE_TYPE, typename EDGE_TYPE>
bool Graph<NODE_TYPE, EDGE_TYPE>::searchBFS(
    uint32_t srcId, uint32_t targetId, std::stack<uint32_t>& path) {
    std::queue<std::shared_ptr<EDGE_TYPE>> queue;
    // Push a dummy edge
    auto dummy = std::make_shared<EDGE_TYPE>(srcId, srcId, 0);
    assert(dummy != nullptr);
    queue.push(dummy);
    std::vector<uint32_t> route(mNodes.size(), kInvalidUnignedNumber);
    std::vector<VisitStatus> visit(mNodes.size(), VisitStatus::Unvisited);
    visit[dummy->mToIndex] = VisitStatus::Visited;
    while (!queue.empty()) {
        auto& next_edge = queue.front();
        assert(next_edge != nullptr);
        queue.pop();
        uint32_t to = next_edge->mToIndex;
        uint32_t from = next_edge->mFromIndex;
        route[to] = from;
        if (to == targetId) {
            getSearchPath(route, srcId, targetId, path);
            return true;
        }
        auto next_edge_list = mEdgeListVector[to];
        for (auto& edge : next_edge_list) {
            assert(edge != nullptr);
            if (visit[edge->mToIndex] == VisitStatus::Unvisited) {
                queue.push(edge);
                visit[edge->mToIndex] = VisitStatus::Visited;
            }
        }
    }
    return false;
}

template <typename NODE_TYPE, typename EDGE_TYPE>
std::shared_ptr<EDGE_TYPE> Graph<NODE_TYPE, EDGE_TYPE>::getEdge(uint32_t from, uint32_t to) {
    if (from >= mNodes.size()) {
        // std::__throw_logic_error("Path Id Over Bound!");
        return nullptr;
    }
    if (to >= mNodes.size()) {
        // std::__throw_logic_error("Dest Id Over Bound!");
        return nullptr;
    }
    auto pathList = mEdgeListVector[from];
    for (auto ele : pathList) {
        if (ele->mFromIndex == from && ele->mToIndex == to) {
            return ele;
        }
    }
    return nullptr;
}

template <typename NODE_TYPE, typename EDGE_TYPE>
int Graph<NODE_TYPE, EDGE_TYPE>::addNode(const std::shared_ptr<NodeType>& node) {
    assert(node != nullptr);
    mNodes.push_back(node);
    uint32_t index = mNodes.size() - 1;
    node->setIndex(index);
    mEdgeListVector.resize(mNodes.size());
    return index;
}

template <typename NODE_TYPE, typename EDGE_TYPE>
void Graph<NODE_TYPE, EDGE_TYPE>::addEdge(uint32_t srcId, uint32_t destId, int cost) {
    assert(srcId < mNodes.size() && destId < mNodes.size());
    addUnidirectionalEdge(srcId, destId, cost);
    addUnidirectionalEdge(destId, srcId, cost);
}

template <typename NODE_TYPE, typename EDGE_TYPE>
void Graph<NODE_TYPE, EDGE_TYPE>::shortestPath(uint32_t srcId) {
    // Please Implement the Algorithmn
    // To Do:
    if (srcId >= mNodes.size()) return;

    // Get ready for a map to calculate visit info.
    uint32_t node_count = mNodes.size();
    std::vector<int> visit_info(mNodes.size(), kInvalidSignedNumber);
    visit_info[srcId] = 0;
    std::vector<int> temp_list_record(mNodes.size(), kInvalidSignedNumber);
    for (int i = 1; i < mNodes.size(); ++i) {
        for (const auto& node : mNodes) {
            auto index = node->getIndex();
            if (visit_info[index] != kInvalidSignedNumber) {
                temp_list_record[index] = kInvalidSignedNumber;
                continue;
            }
            auto edges = mEdgeListVector[index];
            int min_cost = kInvalidSignedNumber;
            for (auto edge : edges) {
                assert(edge != nullptr);
                auto to = edge->mToIndex;
                if (visit_info[to] != kInvalidSignedNumber) {
                    int toDistance = edge->mCost;
                    min_cost = std::min(toDistance + visit_info[to], min_cost);
                }
            }
            temp_list_record[index] = min_cost;
        }
        auto min_element = std::min_element(temp_list_record.begin(), temp_list_record.end());
        auto min_element_index = std::distance(temp_list_record.begin(), min_element);
        visit_info[min_element_index] = *min_element;
    }

    for (int i = 0; i < visit_info.size(); ++i) {
        std::cout << "Shortest Distance From Node [" << srcId << "] to "
                  << "Node [" << i << "]: " << visit_info[i] << std::endl;
    }
}

template <typename NODE_TYPE, typename EDGE_TYPE>
void Graph<NODE_TYPE, EDGE_TYPE>::dumpGraphInfo() {
    int index = 0;
    for (auto node : mNodes) {
        assert(node != nullptr);
        std::cout << "Node: " << node->getName() << " Index: " << node->getIndex() << "-->";
        auto edges = mEdgeListVector[node->getIndex()];
        for (auto edge : edges) {
            assert(edge != nullptr);
            std::cout << "[Fr: " << edge->mFromIndex << " To: " << edge->mToIndex
                      << " Co: " << edge->mCost << "]-->";
        }
        std::cout << "nullptr" << std::endl;
    }
}

template <typename NODE_TYPE, typename EDGE_TYPE>
void Graph<NODE_TYPE, EDGE_TYPE>::addUnidirectionalEdge(uint32_t srcId, uint32_t destId, int cost) {
    assert(srcId < mNodes.size() && destId < mNodes.size());
    if (getEdge(srcId, destId) == nullptr) {
        std::shared_ptr<EDGE_TYPE> ele = std::make_shared<EDGE_TYPE>(srcId, destId, cost);
        ele->mFromIndex = srcId;
        ele->mToIndex = destId;
        ele->mCost = cost;
        mEdgeListVector[srcId].push_back(ele);
    }
}