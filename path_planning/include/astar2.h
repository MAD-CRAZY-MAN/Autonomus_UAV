#ifndef _ASTAR2_H_
#define _ASTAR2_H_

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
#include <queue>

class Node {
    public:
        Eigen::Vector3i index;
        Eigen::Vector3d position;
        double g_score, f_score;
        Node* parent;

        Node() {
            parent = NULL;
        }
        ~Node(){};
};
typedef Node* NodePtr;

class NodeComparator0 {
    public:
        bool operator()(NodePtr node1, NodePtr node2) {
            return node1->f_score > node2->f_score;
        }
};

class Astar {
    public:
        Astar();
        ~Astar();
        enum {REACH_END = 1, NO_PATH = 2 };

        void init();
        void reset();

        int search(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt);
        
        //setResolution
        //pathLength

        std::vector<Eigen::Vector3d> getPath();
        std::vector<Eigen::Vector3d> getVisited();
        //get EarlyTerminateCost

        double _lambda_heu;
        double _max_search_time;
    
    private:
        //backtrack
        void posToIndex(const Eigen::vector3d& pt, Eigen::Vector3i& idx);
        double getDiagHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2);
        double getEuclHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2);

        vector<NodePtr> _path_node_pool;
        int use_node_num, iter_num;
        std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_; //가장 작은 비용 찾기 용 open set
        std::unordered_map<Eigen::vector3i, NodePtr, matrix_hash<Eigen::Vector3i>> _open_set_map; //open set중 노드 검색용 map (hash)
        std::unordered_map<Eigen::Vector3i, int, matrix_hash<Eigen::Vector3i>> _close_set_map;
        std::vector<Eigen::Vector3d> _path_nodes;
        //double early_terminate_cost;

        //EDTEnvironment::Ptr _edt_env;

        double _margin;
        int _allocate_num;
        double _tie_breaker;
        double _resolution, _inv_resolution;
        Eigen::Vector3d _map_size_3d, _origin;
};

#endif