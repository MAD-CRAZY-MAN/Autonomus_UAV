#include <path_planning/astar2.h>

using namespace std;
using namespace Eigen;

Astar::Astar() {
}

Astar::~Astar() {
    for (int i = 0; i < _allocate_num; i++)
        delete _path_node_pool[i];
}

void Astar::init() {
    _resolution  = 0.1;
    _lambda_heu = 5.0;
    _max_search_tim = 1;
    _allocate_num = 100000;

    _tie_breaker = 1.0 + 1.0 / 1000;

    _inv_resolution = 1.0 / _resolution;

    _path_node_pool.resize(_allocate_num);
    for (int i = 0; i< _allocate_num; i++) {
        _path_node_pool[i] = new Node;
    }

    _use_node_num = 0;
    _iter_num = 0
}

int Astar::search(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt) {
    NodePtr cur_node = _path_node_pool[0];
    cur_node->parent = NULL;
    cur_node->position = start_pt;
    posToIndex(start_pt, cur_node->index);
    cur_node->g_score = 0.0;
    cur_node->f_score = _lambda_heu * getDiagHeu(cur_node->position, end_pt);

    Eigen::Vector3i end_index;
    posToIndex(end_pt, end_index);

    _open_set.push(cur_node);
    _open_set_map.insert(make_pair(cur_node->index, cur_node));
    _use_node_num += 1;

    //const auto t1 = ros2::time::now();

    while (!_open_set.empty()) {
        cur_node = _open_set.top();

        bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
                         abs(cur_node->index(1) - end_index(1)) <= 1 &&
                         abs(cur_node->index(2) - end_index(2)) <= 1
        if(reach_end) {
            backtrack(cur_node, end_pt);
            return REACH_END;
        }

        //early termination if time up

        _open_set.pop();
        _open_set_map.erase(cur_node->index);
        _close_set_map.insert(make_pair(cur_node->index, 1));
        iter_num += 1;

        Eigen::Vector3d cur_pos = cur_node->position;
        Eigen::Vector3d nbr_pos;
        Eigen::Vector3d step;

        for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
            for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)   
                for (double dz = -resolution_; dz <= resolution_ + 1e-3; dz += resolution_) {
                    step << dx, dy, dz;
                    if (step.norm() < 1e-3) continue; // 0, 0, 0 방지
                    
                    nbr_pos = cur_pos + step;

                    // check safety

                    // check not in close set
                    Eigen::Vector3i nbr_idx;
                    posToIndex(nbr_pos, nbr_idx);
                    if (_close_set_map.find(nbr_idx) != _close_set_map.end()) continue; //현재 노드 주변의 이웃 노드가 이미 close set에 있으므로 탐색안함.

                    NodePtr neighbor;
                    double tmp_g_score = step.norm() + cur_node->g_score;
                    auto node_iter = _open_set_map.find(nbr_idx);
                    if (node_iter == _open_set_map.end()) { //open set에도 없으므로 새로 할당함

                    } else if (tmp_g_score < node_iter->second->g_score) { //이웃 노드가 open set에 있으나, 비용이 더 적다면 parent를 바꿈
                        neighbor = node_iter->second;
                    }
                    else   
                        continue;

                    neighbor->parent = cur_node;
                    neighbor->g_score = tmp_g_score;
                    neighbor->f_score = tmp_g_score + _lambda_heu * getDiagHeu(nbr_pos, end_pt);
                    _open_set.push(neighbor);
                    _open_set_map[nbr_idx] = neighbor;


                }
    }
}

void Astar::backtrack(const NodePtr& end_node, const Eigen::Vector3d& end) {
    _path_nodes.push_back(end); //최종 노드 위치가 실제 goal과 차이가 나므로 기존 goal 위치를 먼저 넣음
    _path_nodes.push_back(end_node->position);
    NodePtr cur_node = end_node;
    while (cur_node->parent != NULL) {
        cur_node = cur_node->parent;
        _path_nodes.push_back(cur_node->position);
    }
    reverse(_path_nodes.begin(), _path_nodes.end());
}

double Astar::getDiagHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2) {
    double dx = fabs(x1(0) - x2(0));
    double dy = fabs(x1(1) - x2(1));
    double dz = fabs(x1(2) - x2(2));
    double h;
    double diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx < 1e-4) {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy < 1e-4) {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz < 1e-4) {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return tie_breaker_ * h;
}

double Astar::getEuclHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2) {
  return tie_breaker_ * (x2 - x1).norm();
}


