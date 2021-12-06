# 启动mission
gnome-terminal -t "mission" -x bash -c "roslaunch mission  mission_sample.launch; exec bash"

sleep 2s

# 启动px4_bridge
gnome-terminal -t "mission" -x bash -c "roslaunch px4_bridge px4_bridge.launch; exec bash"

sleep 2s

# 启动规划 
gnome-terminal -t "mission" -x bash -c "roslaunch plan_manage kino_replan_gazebo.launch; exec bash"
