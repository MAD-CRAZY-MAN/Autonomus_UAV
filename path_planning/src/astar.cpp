#include <astar.h>

using namespace Eigen;

Astar::~Astar() {
    for (int i = 0; i < allocate_num; i++) {
        delete path_node_pool[i];
    }
}

int Astar::search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt) {

    start_pt(0) = 0.0;
    start_pt(1) = 0.0;
    start_pt(2) = 0.0;

    end_pt(0) = 5.9;
    end_pt(1) = 3.9;
    end_pt(2) = 1.9;

    NodePtr cur_node = path_node_pool[0];
    cur_node->parent = NULL;
    cur_node->position = start_pt;
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
            cout << "allocate_num: " << use_node_num << endl;
            cout << "iter num: " << iter_num << endl; 

            return REACH_END;
        }

        open_set.pop();
        cur_node->node_state = IN_CLOSE_SET;
        cout << "pop open set" << endl;
        cout << "close pos: " << fixed << cur_node->position << '\n' << endl;
        iter_num += 1; //close set 탐색 횟수

        Eigen::Vector3d cur_pos = cur_node->position;
        Eigen::Vector3d pro_pos;

        Eigen::Vector3d d_pos;
        
        for (double dx = -resolution; dx <= resolution + 1e-3; dx += resolution) //resolution == 0.1
            for (double dy = -resolution; dy <= resolution + 1e-3; dy += resolution)
                for (double dz = -resolution; dz <= resolution + 1e-3; dz += resolution) { //resolution == merters/pixel, 0.1 == 10cm
                    d_pos << dx, dy, dz;

                    // if (d_pos.norm() < 1e-3) 
                    //     continue; //1e-3 == 0.001 //L2 norm, 유클리드 노름 //current position 제외
                    
                    pro_pos = cur_pos + d_pos;

                    cout << "neighbor" <<endl;
                    cout << pro_pos << '\n' << endl;
                    
                    if (pro_pos(0) <= origin(0) || pro_pos(0) >= map_size_3d(0) ||
                        pro_pos(1) <= origin(1) || pro_pos(1) >= map_size_3d(1) ||
                        pro_pos(2) <= origin(2) || pro_pos(2) >= map_size_3d(2)) {
                         cout << "outside map" <<endl;
                        // cout << pro_pos << '\n' << endl;
                        // map을 벗어나는지 확인
                        continue;
                    }

                    Eigen::Vector3i pro_id = posToIndex(pro_pos);
                    NodePtr pro_node = expanded_nodes.find(pro_id);

                    if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET) {
                        cout << "in close set" <<endl;
                        continue;
                    }
        
                    // collision free

                    // compute cost
                    double tmp_g_cost, tmp_f_cost;
                    tmp_g_cost = d_pos.squaredNorm() + cur_node->g_cost;
                    tmp_f_cost = tmp_g_cost + lambda_heu * getEuclHeu(pro_pos, end_pt); //f = g + h, 기존 코드는 유클리드 계산에 lambda_heu(5)를 곱함. Heuristic값에 가중치를 주는 것 같음. 테스트 후 수정

                    if (pro_node == NULL) { //이전에 탐색하지 않았다면 계산한 cost 추가하고 open set에 넣음
                        pro_node = path_node_pool[use_node_num];
                        pro_node->index = pro_id;
                        pro_node->position = pro_pos;
                        pro_node->f_cost = tmp_f_cost;
                        pro_node->g_cost = tmp_g_cost;
                        pro_node->parent = cur_node;
                        pro_node->node_state = IN_OPEN_SET;

                        open_set.push(pro_node);

                        expanded_nodes.insert(pro_id, pro_node); //insert key, value to expanded hash table
                        cout << "insert open set" << endl;
                        cout << fixed << pro_node->position << '\n' <<endl;

                        use_node_num += 1; //메모리에 저장된(탐색한) node 수

                        if (use_node_num == allocate_num) { //allocate_num == 100000
                            cout << "[Astar] run out of memory" << endl;
                            return NO_PATH;
                        }
                    }
                    else if (pro_node->node_state == IN_OPEN_SET) { //이미 open set에 있다면 
                        cout << "already" << endl;
                        cout << pro_id << endl;
                        if (tmp_g_cost < pro_node->g_cost) { //현재 cost와 이전 cost를 비교하여, 현재 cost가 적다면 cost와 parent를 변경함
                            pro_node->position = pro_pos;
                            pro_node->f_cost = tmp_f_cost;
                            pro_node->g_cost = tmp_g_cost;
                            pro_node->parent = cur_node;
                        }
                    } else {
                        cout << "error type in searching" << endl;
                    }
                }
    }
    //open set empty, no path
    cout << "[Astar] open set empty, iter num: " << iter_num << endl;
    cout << "allocate_num: " << use_node_num << endl;
    return NO_PATH;
}

double Astar::getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) {
    return 1.0001 * (x2 - x1).norm(); //tie breaker 없음. 정확히 왜 필요한지 아직 모르겟어서 뺌. tie breaker = 1.0001
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
    cout << "astar init" <<endl;
    //param
    resolution = 0.1;
    inv_resolution = 1.0 / resolution; //resolution == 0.1, inv_resolution == 10
    allocate_num = 100000;
    lambda_heu = 5.0;

    //map
    //edt_environment->getMapRejion(origin, map_size_3d);
    origin(0) = 0;
    origin(1) = 0;
    origin(2) = 0;
    map_size_3d(0) = 100;
    map_size_3d(1) = 100;
    map_size_3d(2) = 100;
    
    //init
    path_node_pool.resize(allocate_num);
    for(int i = 0; i < allocate_num; i++) {
        path_node_pool[i] = new Node;
    }

    use_node_num = 0;
    iter_num = 0;
    return;
}

void Astar::reset() {
    expanded_nodes.clear();
    path_nodes.clear();

    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> empty_queue;
    open_set.swap(empty_queue); //nsh?

    for(int i = 0; i < use_node_num; i++) {
        NodePtr node = path_node_pool[i];
        node->parent = NULL;
        node->node_state = NOT_EXPAND;
    }

    use_node_num = 0;
    iter_num = 0;
}

Eigen::Vector3i Astar::posToIndex(Eigen::Vector3d pt) {
    //Vector3i idx = ((pt - origin) * inv_resolution).array().floor().cast<int>();
    Vector3i idx;
    idx << static_cast<int>(round((pt(0) - origin(0)) * inv_resolution)), 
           static_cast<int>(round((pt(1) - origin(1)) * inv_resolution)),
           static_cast<int>(round((pt(2) - origin(2)) * inv_resolution));
    //floor = 주어진 값보다 크지 않은 가장 가까운 정수
    //array는 계수별 연산에 대한 쉬운 엑세스 제공. matrix, vector는 선형 대수 연산에 대한 쉬운 엑세스 제공
    //array는 곱셈 시 계수 별로 곱셈하도록 함 (내적, 외적이 아닌 일반 배열로 처리)
    return idx;
}

std::vector<Eigen::Vector3d> Astar::getPath() {
    vector<Eigen::Vector3d> path;
    for (unsigned long i = 0; i < path_nodes.size(); ++i) { // int -> unsigned long
        path.push_back(path_nodes[i]->position);
    }
    return path;
}