#ifndef _ASTAR_REPLAN_FSM_H_
#define _ASTAR_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <astar.h>
#include <plan_manager.h>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <functional>
#include <iostream>
#include <string>

using namespace std;
using namespace std::chrono_literals;

class AstarReplanFSM : public rclcpp::Node {

    public:
        AstarReplanFSM(): Node("planning_node")
        {}
        ~AstarReplanFSM(){}

        void init();

    private:
        enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW };

        PlannerManager::Ptr _planner_manager;

        // planning data
        bool _trigger, _have_target, _have_odom;
        FSM_EXEC_STATE _exec_state;

        Eigen::Vector3d _start_pt, _end_pt;
        Eigen::Vector3d _odom_pos;

        //helper functions
        bool callAstarReplan();

        void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
        
        // ROS2 functions
        void execFSMCallback();
        void checkCollisionCallback();

        //
        rclcpp::TimerBase::SharedPtr _timer;

};

#endif