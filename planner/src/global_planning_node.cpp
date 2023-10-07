#include "global_planner.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_planning_node");
  ros::NodeHandle nh("~");

  GlobalPlanner* gp = new GlobalPlanner();
  gp->init(nh);

  // std::thread visualization_thread(&GlobalPlanner::visGridMap, gp); //TODO: visualization thread

  while (ros::ok())
  {
    auto spinOnce_start_time = std::chrono::steady_clock::now();
    ros::spinOnce();
    auto spinOnce_end_time = std::chrono::steady_clock::now();

    auto motionModeDetect_start_time = std::chrono::steady_clock::now();
    gp->motionModeDetect();
    auto motionModeDetect_end_time = std::chrono::steady_clock::now();

    auto callPlanner_start_time = std::chrono::steady_clock::now();
    gp->callPlanner();
    auto callPlanner_end_time = std::chrono::steady_clock::now();

    auto total_time = std::chrono::duration_cast<std::chrono::duration<double>>(callPlanner_end_time - spinOnce_start_time);
    if(total_time.count() > 1e-3)
    {
      ROS_WARN("main loop time: %f", total_time.count());
      gp->log_data_.main_loop_time = total_time.count();
      gp->log_data_.updated = true;
    }
    if(gp->run_time_log_)
      gp->plotLog();
  }

  // visualization_thread.join();

  if(!ros::ok())
    gp->exit();

  return 0;
}
