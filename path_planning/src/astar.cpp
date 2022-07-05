#include "astar.h"

using namespace std;
using namespace Eigen;

Astar::~Astar() {

}

int Astar::search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt) {
    NodePtr cur_node = path_node_pool[0];
    cur_node->parent = NULL;
    cur_node->index = posToIndex(start_pt);
    cur_node->g_cost = 0.0;

    Eigen::Vector3i end_index = posToIndex(end_pt);
    
    cur_node->f_cost = lambda_heu * getEuclHeu(cur_node->position, end_pt);
    cur_node->node_state = IN_OPEN_SET;

    open_set.push(cur_node);
    use_node_num += 1;

    expanded_nodes.insert(cur_node->index, cur_node);

    NodePtr neighbor = NULL;
    NodePtr terminate_node = NULL;

    while(!open_set.empty()) {
        cur_node = open_set.top();

        bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
                         abs(cur_node->index(1) - end_index(1)) <= 1 &&
                         abs(cur_node->index(2) - end_index(2)) <= 1;
        if (reach_end) {
            terminate_node = cur_node;
            retrievePath(terminate_node);
            has_path = true;

            return REACH_END;
        }

        open_set.pop();
        cur_node->node_state = IN_CLOSE_SET;
        iter_num += 1;

        Eigen::Vector3d cur_pos = cur_node->position;
        Eigen::Vector3d pro_pos;

    }

    return NO_PATH;
}

void Astar::retrievePath(NodePtr end_node) {
    NodePtr cur_node = end_node;
    path_nodes.push_back(cur_node);

    while(cur_node->parent != NULL) {
        cur_node = cur_node->parent;
        path_nodes.push_back(cur_node);
    }

    reverse(path_nodes.begin(), path_nodes.end());
}

void Astar::reset() {
    expanded_nodes.clear();
    path_nodes.clear();

    for(int i = 0; i < use_node_num; i++) {
        NodePtr node = path_node_pool[i];
        node->parent = NULL;
        node->node_state = NOT_EXPAND;
    }

    use_node_num = 0;
    iter_num = 0;
}

Eigen::Vector3i Astar::posToIndex(Eigen::Vector3d pt) {
    Vector3i idx;// = ((pt -origin))
    return idx;
}