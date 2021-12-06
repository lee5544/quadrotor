gnome-terminal -t "mission" -x bash -c "roslaunch realsense2_camera rs_depth.launch; exec bash"

sleep 2s

gnome-terminal -t "mission" -x bash -c "rosrun perception detector.py; exec bash"
