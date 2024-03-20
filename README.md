How to run autonomy stack, intended for use alonside AWSIM 

1)	clone main branch using http or ssh
2)	git submodule init
3)	git submodule update 
4)	colcon build --symlink-install
5)	source install/setup.bash
6)	ros2 launch simulator launch_sim.launch.py 

