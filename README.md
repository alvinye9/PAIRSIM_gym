How to run autonomy stack, intended for use alonside AWSIM 

1)	Pull main branch
2)	git submodule init
3)	git submodule update 
4)	Manually clone the submodule ./src/sim-msgs from https://github.com/autonomalabs/sim-msgs
5)	colcon build --symlink-install
6)	source install/setup.bash
7)	ros2 launch simulator launch_sim 
