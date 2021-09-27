# 启动gazebo仿真
gnome-terminal -t "gazebo_outdoor_px4" -x bash -c "roslaunch sitl_launch gazebo_outdoor_px4.launch; exec bash"

sleep 5s

# 启动mission
gnome-terminal -t "mission" -x bash -c "roslaunch mission mission_sample.launch; exec bash"

sleep 2s

# 启动px4_bridge
gnome-terminal -t "px4_bridge_sitl" -x bash -c "roslaunch sitl_launch px4_bridge_sitl.launch; exec bash"

sleep 2s

# 启动规划 
gnome-terminal -t "kino_replan_sitl" -x bash -c "roslaunch sitl_launch kino_replan_sitl.launch; exec bash"
