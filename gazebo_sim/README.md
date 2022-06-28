# Gazebo_Sim

### Requirements

- ROS2-Dashing or ROS2-fosy 



### Quick Start

1. install gazebo 11
2. install PX4-Autopilot
   - cd Home_dir
   - git clone https://github.com/the-peach-drone/PX4-Autopilot
   - cd PX4-Autopilot
   - make px4_sitl_rtps
3. gazebo ros libaries
   - sudo apt install ros-foxy-gazebo11-*
   - sudo apt install ros-foxy-ros-core
4. urdf, xacro parser
   - sudo apt install ros-foxy-urdf
   - sudo apt install ros-foxy-xacro
5. repository clone
   - cd ~/ros2_ws/src
   - git clone https://github.com/MAD-CRAZY-MAN/Autonomus_UAV
6. build
   - colcon build --symlink-install
7. start 
   - ros2 launch gazebo_sim daedeok_police_office_sim.launch.py
8. check topic
   - ros2 topic list
     - /bronco/depthCamera
     - /bronco/stereoCamera
     - /bronco/fisheyeCamera
     - /bronco/hiresCamera
9. Run GCS and control the drone
   - install QGC
   - Run QGC
   - Use joystick or command
