#include <astar.h>

using namespace Eigen;

Astar::~Astar() {
    for (int i = 0; i < allocate_num; i++) {
        delete path_node_pool[i];
    }
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
        iter_num += 1; //close set 탐색 횟수

        Eigen::Vector3d cur_pos = cur_node->position;
        Eigen::Vector3d pro_pos;
        Eigen::Vector3d d_pos;

        for (double dx = -resolution; dx <= resolution + 1e-3; dx += resolution) //resolution == 0.1
            for (double dy = -resolution; dy <= resolution + 1e-3; dy += resolution)
                for (double dz = -resolution; dz <= resolution + 1e-3; dz += resolution) { //resolution == merters/pixel, 0.1 == 10cm
                    d_pos << dx, dy, dz;

                    if (d_pos.norm() < 1e-3) continue; //1e-3 == 0.001 //L2 norm, 유클리드 노름

                    pro_pos = cur_pos + d_pos;

                    if (pro_pos(0) <= origin(0) || pro_pos(1) >= map_size_3d(0) ||
                        pro_pos(1) <= origin(1) || pro_pos(2) >= map_size_3d(2) ||
                        pro_pos(2) <= origin(2) || pro_pos(2) >= map_size_3d(2)) {
                        //map을 벗어나는지 확인
                        continue;
                    }

                    Eigen::Vector3i pro_id = posToIndex(pro_pos);
                    NodePtr pro_node = expanded_nodes.find(pro_id);

                    if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET) {
                        continue;
                    }
        
                    // collision free

                    // compute cost
                    double tmp_g_cost, tmp_f_cost;
                    tmp_g_cost = d_pos.squaredNorm() + cur_node->g_cost;
                    tmp_f_cost = tmp_g_cost + getEuclHeu(pro_pos, end_pt); //f = g + h, 기존 코드는 유클리드 계산에 lambda_heu(5)를 곱함. Heuristic값에 가중치를 주는 것 같음. 테스트 후 수정

                    if (pro_node == NULL) { //이전에 탐색하지 않았다면 계산한 cost 추가하고 open set에 넣음
                        pro_node = path_node_pool[use_node_num];
                        pro_node->index = pro_id;
                        pro_node->f_cost = tmp_f_cost;
                        pro_node->g_cost = tmp_g_cost;
                        pro_node->parent = cur_node;
                        pro_node->node_state = IN_OPEN_SET;

                        open_set.push(pro_node);
                        expanded_nodes.insert(pro_id, pro_node); //insert key, value to expanded hash table

                        use_node_num += 1; //메모리에 저장된(탐색한) node 수
                        if (use_node_num == allocate_num) { //allocate_num == 100000
                            //run out of memory
                            return NO_PATH;
                        }
                    }
                    else if (pro_node->node_state == IN_OPEN_SET) { //이미 open set에 있다면 
                        if (tmp_g_cost < pro_node->g_cost) { //현재 cost와 이전 cost를 비교하여, 현재 cost가 적다면 cost와 parent를 변경함
                            pro_node->position = pro_pos;
                            pro_node->f_cost = tmp_f_cost;
                            pro_node->g_cost = tmp_g_cost;
                            pro_node->parent = cur_node;
                        }
                    }
                }

    }
    //open set empty, no path
    return NO_PATH;
}

double Astar::getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) {
    return (x1 - x2).norm(); //tie breaker 없음. 정확히 왜 필요한지 아직 모르겟어서 뺌. tie breaker = 1.0001
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

void Astar::init() {
    //parma
    resolution = 0.1;
    inv_resolution = 1.0 / resolution;
    allocate_num = 100000;

    //init
    path_node_pool.resize(allocate_num);
    for(int i = 0; i < allocate_num; i++) {
        path_node_pool[i] = new Node;
    }

    use_node_num = 0;
    iter_num = 0;
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
    Vector3i idx = ((pt - origin) * inv_resolution).array().floor().cast<int>();
    //floor = 주어진 값보다 크지 않은 가장 가까운 정수
    //array는 계수별 연산에 대한 쉬운 엑세스 제공. matrix, vector는 선형 대수 연산에 대한 쉬운 엑세스 제공
    return idx;
}

std::vector<Eigen::Vector3d> Astar::getPath() {
    vector<Eigen::Vector3d> path;
    for (unsigned long i = 0; i < path_nodes.size(); ++i) { // int -> unsigned long
        path.push_back(path_nodes[i]->position);
    }
    return path;
}