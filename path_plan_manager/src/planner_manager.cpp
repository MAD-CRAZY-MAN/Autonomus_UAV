#include <plan_manager.h>

PlannerManager::PlannerManager() {}
PlannerManager::~PlannerManager() {}

bool PlannerManager::astarReplan(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt) {
    _path_finder->reset();

    int status = _path_finder->search(start_pt, end_pt);

    if (status == Astar::NO_PATH) {
        cout << "search fail!" <<endl;   
    }
    else {
        cout << "search success" << endl;
    }

    //plan_data.path = path_finder->getTraj(0.01);
    return true; //modify
}