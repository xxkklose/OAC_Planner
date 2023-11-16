#include "oac_mapping.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "oac_mapping_node");
    Mapping mapping;
    mapping.init();
    mapping.process();
    return 0;
}
