#ifndef _ASTAR_REPLAN_FSM_H_
#define _ASTAR_REPLAN_FSM_H_

#include <Eigen/Eigen>

#include <iostream>
#include <string>

using namespace std;

class AstarReplanFSM {

    public:
        AstarReplanFSM(){}
        ~AstarReplanFSM(){}

    private:
        enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW };

        // planning data
        bool _trigger, _have_target, _have_odom;
        FSM_EXEC_STATE _exec_state;

        Eigen::Vector3d _start_pt, _end_pt;

        //helper functions
        void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
        
        // ROS2 functions
        void execFSMCallback();
        void checkCollisionCallback();

};

#endif