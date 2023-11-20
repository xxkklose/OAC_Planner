#include "oac_global_planner.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_planning_node");
  ros::NodeHandle nh("~");

  GlobalPlanner gp;
  gp.init(nh);
  gp.process();

  return 0;
}
