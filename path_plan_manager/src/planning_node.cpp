#include <iostream>
#include <string>
#include <chrono>

#include <astar_replan_fsm.h>

#include <rclcpp/rclcpp.hpp>

using namespace std;
using namespace std::chrono_literals;

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);

    auto fsm = make_shared<AstarReplanFSM>();
    fsm->init();
    rclcpp::spin(fsm);
    rclcpp::shutdown();
    
    return 0;
}