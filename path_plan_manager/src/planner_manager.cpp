#include <planner_manager.h>

PlannerManager::PlannerManager() {
    _path_finder = std::make_unique<Astar>();
    _path_finder->init();
}

PlannerManager::~PlannerManager() {}

void PlannerManager::init() {
    cout <<"plan manager init" <<endl;
    
    _path_finder->init();
}

bool PlannerManager::astarReplan(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt) {
    cout << "start search1" << endl;

    _path_finder->reset();

    cout << "start search2" << endl;
    int status = _path_finder->search(start_pt, end_pt);

    if (status == Astar::NO_PATH) {
        cout << "search fail!" <<endl;   
        return false;
    }
    else {
        cout << "search success" << endl;

        std::vector<Eigen::Vector3d> test = _path_finder->getPath();
        
    
        for (auto iter = test.begin(); iter!= test.end(); iter++) { // int -> unsigned long
            cout << *iter << endl;
        }
        return true; //modify
    }
    

    //plan_data.path = path_finder->getTraj(0.01);
}