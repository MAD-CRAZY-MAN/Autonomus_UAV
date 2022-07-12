#include <astar_replan_fsm.h>

AstarReplanFSM::AstarReplanFSM(): Node("planning_node")
{
    _planner_manager = std::make_unique<PlannerManager>();
}

void AstarReplanFSM::init() {
    _exec_state = FSM_EXEC_STATE::GEN_NEW_TRAJ;
    _have_target = true;
    _have_odom = true;
    cout <<"fsm init" <<endl;

    _planner_manager->init();
    _timer = this->create_wall_timer(1s, std::bind(&AstarReplanFSM::execFSMCallback, this));
}

void AstarReplanFSM::execFSMCallback() {
    cout << "FSM start" << endl;

    switch (_exec_state) {
        case INIT: {
            if (!_have_odom) {
                return;
            }
            if (!_trigger) {
                return;
            }
            changeFSMExecState(WAIT_TARGET, "FSM");
            break;
        }
        
        case WAIT_TARGET: {
            if (!_have_target) {
                return;
            }
            else {
                changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            }
            break;
        }

        case GEN_NEW_TRAJ: {
            _start_pt = _odom_pos;

            //rotate
            cout <<"GEN_NEW_TRAJ" <<endl;
            bool success = callAstarReplan();
            if (success) {
                changeFSMExecState(EXEC_TRAJ, "FSM");
            }
            else {
                changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            }
            break;
        }

        case EXEC_TRAJ: {
            break;
        }

        case REPLAN_TRAJ: {
            
            bool success = callAstarReplan();
            if (success) {
                changeFSMExecState(EXEC_TRAJ, "FSM");
            } else {
                changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            }
            break;
        }
    }
}

bool AstarReplanFSM::callAstarReplan() {
    
    bool success = _planner_manager->astarReplan(_start_pt, _end_pt);

    if (success) {

        return true;
    }
    else {
        return false;
    }
}

void AstarReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
    cout << "test " <<endl;
    string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
    int    pre_s        = int(_exec_state);
    _exec_state = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}