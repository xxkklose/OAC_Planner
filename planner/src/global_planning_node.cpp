#include "global_planner.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_planning_node");
  ros::NodeHandle nh("~");

  GlobalPlanner* gp = new GlobalPlanner();
  gp->init(nh);
  
  while (ros::ok())
  {
    auto start_time = std::chrono::steady_clock::now();
    ros::spinOnce();
    gp->motionModeDetect();
    gp->callPlanner();
    auto end_time = std::chrono::steady_clock::now();
    auto time = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
    // ROS_WARN("main loop time: %f", time.count());
    // ros::Duration(planning_time_horizon).sleep();
  }

  if(!ros::ok())
    gp->exit();

  return 0;
}
