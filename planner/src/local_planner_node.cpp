#include "local_planner.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_planning_node");
    LocalPlanner local_planner;
    local_planner.init();
    local_planner.process();
    return 0;
}