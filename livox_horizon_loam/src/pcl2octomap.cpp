#include <iostream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>


ros::Publisher transform_pub;
void transformCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // ROS_INFO("pcl2octomap shows: %d", msg->height);
    pcl::fromROSMsg(*msg, *cloud);

    octomap::OcTree octree(0.05);

    for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it != cloud->end(); ++it)
    {
        octomap::point3d point(it->x, it->y, it->z);

        octree.updateNode(point, true);
    }

    octree.updateInnerOccupancy();

    octomap_msgs::Octomap outmsg;
    octomap_msgs::binaryMapToMsg(octree, outmsg);

    outmsg.header.stamp = msg->header.stamp;
    outmsg.header.frame_id = msg->header.frame_id;
    transform_pub.publish(outmsg);
    
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pcl2octomap");
    ros::NodeHandle nh;

    ros::Subscriber transform_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 100, transformCallback);
    transform_pub = nh.advertise<octomap_msgs::Octomap>("octomap", 100);

    ros::spin();

    return 0;
}
