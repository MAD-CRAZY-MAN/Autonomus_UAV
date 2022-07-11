#ifndef _PLAN_MANAGER_H_
#define _PLAN_MANAGER_H_

#include <astar.h>
#include <memory>

using namespace std;

class PlannerManager {
    public:
        PlannerManager();
        ~PlannerManager();

        bool astarReplan(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

        typedef unique_ptr<PlannerManager> Ptr;

    private:
        unique_ptr<Astar> _path_finder;
};


#endif