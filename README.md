How to run autonomy stack, intended for use alonside PAIRSIM

1)	clone main branch using http or ssh
2)	colcon build --symlink-install
3)	source install/setup.bash
4)	ros2 launch simulator launch_sim.launch.py 

REQUIREMENTS: 
- PAIRSIM executable (.x86_64)
- Ubuntu 22.04
- ROS2 Humble

