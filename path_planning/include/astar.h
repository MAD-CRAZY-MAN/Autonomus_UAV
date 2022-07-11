#ifndef _ASTAR_H
#define _ASTAR_H

#include <Eigen/Eigen>

#include <iostream>
#include <string>

#include <queue>
#include <unordered_map>
#include <memory>

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'

using namespace std;

class Node {
    public:
        Eigen::Vector3i index;
        Eigen::Vector3d position;
        double g_cost, f_cost;
        Node* parent;
        char node_state;

        Node() {
            parent = NULL;
            node_state = NOT_EXPAND;
        }
        ~Node(){};
};
typedef Node* NodePtr;

class NodeComparator0 { //비용 비교 함수. open set priority queue에서 f cost가 가장 적은 노드를 꺼낼 때 사용함.
    public:
        bool operator()(NodePtr node1, NodePtr node2) {
            return node1->f_cost > node2->f_cost;
        }
};

template <typename T>
struct matrix_hash0 : std::unary_function<T, size_t> {
    std::size_t operator()(T const& matrix) const {
        size_t seed = 0;
        for (size_t i = 0; i < static_cast<size_t>(matrix.size()); ++i) { //static_cast<size_t>, size_t = long unsigne int, Eigen::Matrix<int, 3, 1> = long int
            auto elem = static_cast<size_t>(*(matrix.data() + i));
            seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2); //hash combine
        }
        return seed;
    }
};

class NodeHashTable0 {
    private:
        std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash0<Eigen::Vector3i>> data_3d;
    public:
        void insert(Eigen::Vector3i idx, NodePtr node) {
            data_3d.insert(make_pair(idx, node));
        }

        NodePtr find(Eigen::Vector3i idx) { //insert key, output value
            auto iter = data_3d.find(idx);
            return iter == data_3d.end() ? NULL : iter->second;
        }

        void clear() {
            data_3d.clear();
        }
};

class Astar {
    public:
        Astar(){};
        ~Astar();

        enum { REACH_END = 1, NO_PATH = 2 };

        void init();
        int search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
        void reset();
        
        std::vector<Eigen::Vector3d> getPath(); //fsm에서 쓰는지 확인
        typedef unique_ptr<Astar> Ptr;

    private:
        /* ---------- main data structure ---------- */
        vector<NodePtr> path_node_pool;
        NodeHashTable0 expanded_nodes;
        std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set;
        std::vector<NodePtr> path_nodes; //final generate path
        int use_node_num, iter_num;

        /* ---------- record data ---------- */
        bool has_path = false;

        /* ---------- parameter ---------- */
        /* search */
        double lambda_heu;
        double margin;
        int allocate_num;
        double tie_breaker;
        /* map */
        double resolution;
        double inv_resolution;
        Eigen::Vector3d origin, map_size_3d;

        /* helper */
        Eigen::Vector3i posToIndex(Eigen::Vector3d pt); //index == key
        void retrievePath(NodePtr end_node);

        /* heuristic function */
        double getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
};

#endif