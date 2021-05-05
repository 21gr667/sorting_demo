#include <ros/ros.h>
#include "sorting_demo/object.h"
#include <iostream>
#include <queue>
#include <math.h>

// Moveit headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

using namespace std;

const std::string PLANNING_GROUP = "manipulator";
int serial_port = 0;
bool isDonePicking = true;
double conveyorSpeed = 1000/25.15;
std::vector<double> orientation = {-0.647821, -0.308727, 0.288902, 0.633681};

double deg2rad(double degree) {
    double pi = 3.14159265359;
    return (degree * (pi / 180));
}

class Object {
    public:
        double x;
        double y;
        double z;
        ros::Time stamp;
        std::string type;
};

queue<Object> objects;

void objectCallback(const sorting_demo::object::ConstPtr& msg) {
    Object newObject;
    newObject.x = msg->point.x;
    newObject.y = msg->point.y;
    newObject.z = msg->point.z;
    newObject.stamp = msg->stamp;
    newObject.type = msg->type.c_str();
    objects.push(newObject);
}

void set_gripper_state(bool open) {
    if (open) {
        write(serial_port, "1", 2);
        ROS_INFO("Gripper opened\n");
    } else {
        write(serial_port, "0", 2);
        ROS_INFO("Gripper closed\n");
    }
}

void go_to_joint_position(std::vector<double> joint_goal) {
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group.setMaxAccelerationScalingFactor(1.0);
    move_group.setJointValueTarget(joint_goal);
    move_group.setNumPlanningAttempts(10);
    move_group.plan(plan);
    move_group.execute(plan);
}

void go_to_position(std::vector<double> goal) {

    /*
    ROS_INFO("aaaa");

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = -0.28;
    target_pose1.position.y = -0.01;
    target_pose1.position.z = 0.5;
    move_group.setPoseTarget(target_pose1);
    ROS_INFO("bbbb");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    ROS_INFO("cccc");
    move_group.plan(my_plan);
    ROS_INFO("dddd");
    move_group.move();
    ROS_INFO("eeee");
    */
   
    ROS_INFO("x: %f", goal[0]);
    ROS_INFO("y: %f", goal[1]);
    ROS_INFO("z: %f", goal[2]);

/*
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.x = -0.647821;
    target_pose1.orientation.y = -0.308727;
    target_pose1.orientation.z = 0.288902;
    target_pose1.orientation.w = 0.633681;
    target_pose1.position.x = goal[0];
    target_pose1.position.y = goal[1];
    target_pose1.position.z = goal[2];
    move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    move_group.move();
*/


    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group.setMaxAccelerationScalingFactor(1.0);

    move_group.setPositionTarget(goal[0], goal[1], goal[2]);
    //move_group.setOrientationTarget(-0.647821, -0.308727, 0.288902, 0.633681);
    move_group.setRPYTarget(goal[3], goal[4], goal[5]);

/*
    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = -0.647821;
    target_pose.orientation.y = -0.308727;
    target_pose.orientation.z = 0.288902;
    target_pose.orientation.w = 0.633681;
    target_pose.position.x = goal[0];
    target_pose.position.y = goal[1];
    target_pose.position.z = goal[2];
    move_group.setPoseTarget(target_pose);
    */

    move_group.setNumPlanningAttempts(10);
    move_group.plan(plan);
    move_group.execute(plan);

}

std::vector<double> computePickPosition(Object object) {
    /*
    double x = 0.08 + object.y;
    double y = 0.01;
    double z = - 0.395 + object.z + 0.1;
    */


    double x = object.x;
    double y = object.y;
    double z = object.z;
    return {x, y, z};
}

void goToPickPosition(std::vector<double> position) {
    position.push_back(2.5);
    position.push_back(-1.93);
    position.push_back(0.0);
    go_to_position(position);
}

void waitForObject(Object object) {
    double travelTime = conveyorSpeed/(760 - object.x);

    ros::Duration timeSpend = ros::Time::now() - object.stamp;
    ros::Duration timeLeft = ros::Duration(travelTime) - timeSpend;
    timeLeft.sleep();
}

void pick(std::vector<double> position) {
    std::vector<double> downPick = position;
    downPick.push_back(1.141592);
    downPick.push_back(0.0);
    downPick.push_back(0.0);
    downPick[2] = downPick[2] - 0.02;
    std::vector<double> upPick = position;

    go_to_position(downPick);
    go_to_position(upPick);
}

void goToBucket(string objectType) {

}

void goToDefaultPosition() {

}

void tryPickNext() {
    if (isDonePicking && !objects.empty()) {
        isDonePicking = false;
        Object pickObject = objects.front();
        objects.pop();

        ROS_INFO("X: %f", pickObject.x);
        ROS_INFO("found: %f", pickObject.y);
        ROS_INFO("found: %f", pickObject.z);
        ROS_INFO("found: %f", pickObject.stamp.toSec());
        ROS_INFO("found: %s", pickObject.type.c_str());

        std::vector<double> pickPosition = computePickPosition(pickObject);
        goToPickPosition(pickPosition);
        //set_gripper_state(true);
        //waitForObject(pickObject);
        //pick(pickPosition);
        //goToBucket(pickObject.type.c_str());
        //set_gripper_state(false);
        //goToDefaultPosition();

        isDonePicking = true;
    }
}

void configure_serial_port() {
    serial_port = open("/dev/ttyACM0", O_RDWR);

    struct termios tty;

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 115200
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    ros::Duration(3.0).sleep();
}





void printPose() {
    moveit::planning_interface::MoveGroupInterface group("manipulator");

    geometry_msgs::PoseStamped currentPose = group.getCurrentPose();

    geometry_msgs::Point position = currentPose.pose.position;
    geometry_msgs::Quaternion orientation = currentPose.pose.orientation;

    ROS_INFO("x pos: %f", position.x);
    ROS_INFO("y pos: %f", position.y);
    ROS_INFO("z pos: %f", position.z);

    ROS_INFO("qx pos: %f", orientation.x);
    ROS_INFO("qy pos: %f", orientation.y);
    ROS_INFO("qz pos: %f", orientation.z);
    ROS_INFO("qw pos: %f", orientation.w);

    ROS_INFO("\n\n");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "picking_node");
    ros::AsyncSpinner spinner(4);
    ros::NodeHandle nodeHandle;

    ros::Subscriber subscriber = nodeHandle.subscribe("objects", 1000, objectCallback);

    spinner.start();

    configure_serial_port();

    //go_to_joint_position({deg2rad(176), deg2rad(-95), deg2rad(104), deg2rad(-97), deg2rad(-90), deg2rad(-54)});





    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group.setMaxAccelerationScalingFactor(1.0);

    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = -0.647821;
    target_pose.orientation.y = -0.308727;
    target_pose.orientation.z = 0.288902;
    target_pose.orientation.w = 0.633681;
    target_pose.position.x = -0.441450;
    target_pose.position.y = -0.080358;
    target_pose.position.z = 0.146406;
    move_group.setPoseTarget(target_pose);

    move_group.setNumPlanningAttempts(10);
    move_group.plan(plan);
    move_group.execute(plan);





/*
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        //ros::spinOnce();
        tryPickNext();
        //ROS_INFO("wuhu");
        printPose();
        loop_rate.sleep();
    }
    */

    ros::waitForShutdown();

    close(serial_port);

    return 0;
}

//Get data into array
//loop through array
    //suck
    //pick up
    //go to bucket
    //stop suck
