#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <astar.h>
#include <memory>
#include <iostream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <functional>


using namespace std;

class PlannerManager {
    public:
        PlannerManager();
        ~PlannerManager();

        void init();
        bool astarReplan(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

        typedef unique_ptr<PlannerManager> Ptr;

        
    private:
        std::unique_ptr<Astar> _path_finder;
        
};


#endif