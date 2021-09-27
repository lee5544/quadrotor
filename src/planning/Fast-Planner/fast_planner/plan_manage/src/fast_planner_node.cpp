#include <ros/ros.h>
//ros/ros.h为ros系统基本功能所需的头文件
#include <visualization_msgs/Marker.h>

#include <plan_manage/kino_replan_fsm.h>
#include <plan_manage/topo_replan_fsm.h>

#include <plan_manage/backward.hpp>//??? 


#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;


int main(int argc, char** argv) 
{
  ros::init(argc, argv, "fast_planner_node");
  ros::NodeHandle nh("~");   
        int planner;
        nh.param("planner_node/planner", planner, -1);

        TopoReplanFSM topo_replan;
        KinoReplanFSM kino_replan;

            if (planner == 1) 
            {
            kino_replan.init(nh); 
            } 
            else if (planner == 2)
            {
            topo_replan.init(nh);
            }

    // sleep for a second
    ros::Duration(1.0).sleep();
    
    /**
     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
     * callbacks will be called from within this thread (the main one).  ros::spin()
     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
    ros::spin();


  return 0;
    
}
