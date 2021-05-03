#include <ros/ros.h>
#include "sorting_demo/object.h"
#include <iostream>

void objectCallback(const sorting_demo::object::ConstPtr& msg) {
    ROS_INFO("I heard x: [%f]", msg->point.x);
    ROS_INFO("I heard y: [%f]", msg->point.y);
    ROS_INFO("I heard z: [%f]", msg->point.z);
    ROS_INFO("I heard time: [%d]", msg->stamp.toSec());
    ROS_INFO("I heard: [%s]", msg->type.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "picking_node");

    ros::NodeHandle nodeHandle;

    ros::Subscriber subscriber = nodeHandle.subscribe("objects", 1000, objectCallback);

    ROS_INFO("Hello World!");

    ros::spin();

    return 0;
}

//Get data into array
//loop through array
    //suck
    //pick up
    //go to bucket
    //stop suck
