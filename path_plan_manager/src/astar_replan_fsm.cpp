#include <astar_replan_fsm.h>

void AstarReplanFSM::init() {
    _exec_state = FSM_EXEC_STATE::INIT;
    _have_target = false;
    _have_odom = false;
    
    _timer = this->create_wall_timer(1s, std::bind(&AstarReplanFSM::execFSMCallback, this));
}

void AstarReplanFSM::execFSMCallback() {
    cout << "test" << endl;

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
    _exec_state = new_state;
}