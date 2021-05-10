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
double conveyorSpeed = 1/25.15;
moveit::planning_interface::MoveGroupInterface *move_group;
double pickYPosition = 0.3;

double deg2rad(double degree) {
    double pi = 3.14159265359;
    return (degree * (pi / 180));
}

std::vector<double> defaultJointGoal = {deg2rad(-23), deg2rad(-90), deg2rad(-97), deg2rad(-82), deg2rad(87), deg2rad(0)};
std::vector<double> redJointGoal = {deg2rad(-25.0), deg2rad(-131.0), deg2rad(-38), deg2rad(-75), deg2rad(89), deg2rad(61)};
std::vector<double> yellowJointGoal = {deg2rad(-7.0), deg2rad(-131.0), deg2rad(-28), deg2rad(-77), deg2rad(87), deg2rad(61)};
std::vector<double> blueJointGoal = {deg2rad(12.0), deg2rad(-115.0), deg2rad(-56), deg2rad(-93), deg2rad(89), deg2rad(61)};

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
    newObject.x = msg->point.x / 1000;
    newObject.y = msg->point.y / 1000;
    newObject.z = msg->point.z / 1000;
    newObject.stamp = msg->stamp;
    newObject.type = msg->type.c_str();
    objects.push(newObject);
}

void set_gripper_state(bool open) {
    if (open) {
        write(serial_port, "1", 2);
        ROS_INFO("Gripper opened");
    } else {
        write(serial_port, "0", 2);
        ROS_INFO("Gripper closed");
    }
}

void go_to_joint_position(std::vector<double> joint_goal) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group->setMaxAccelerationScalingFactor(1.0);
    move_group->setJointValueTarget(joint_goal);
    move_group->setNumPlanningAttempts(10);
    move_group->plan(plan);
    move_group->execute(plan);
}

void go_to_position(std::vector<double> goal) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    move_group->setMaxAccelerationScalingFactor(1.0);

    geometry_msgs::Pose target_pose;

    target_pose.orientation.x = -0.497239;
    target_pose.orientation.y = 0.494919;
    target_pose.orientation.z = 0.503958;
    target_pose.orientation.w = 0.503821;
    target_pose.position.x = goal[0];
    target_pose.position.y = goal[1];
    target_pose.position.z = goal[2];

    move_group->setPoseTarget(target_pose);
    move_group->setNumPlanningAttempts(10);
    move_group->plan(plan);
    move_group->execute(plan);
}

std::vector<double> computePickPosition(Object object) {
    double x = -0.11 - object.x;
    double y = pickYPosition;
    double z = 0.265;
    return {x, y, z};
}

void waitForObject(Object object) {
    double travelTime = (0.780 - pickYPosition - object.y)/conveyorSpeed;

    ROS_INFO("travel time: %f", travelTime);

    ros::Duration timeSpend = ros::Time::now() - object.stamp;

    ros::Duration timeLeft = ros::Duration(travelTime) - timeSpend + ros::Duration(0.8);

    ROS_INFO("witing for %f seconds", timeLeft.toSec());

    if (timeLeft.toSec() > 0.0) {
        timeLeft.sleep();
    }
    
   //ros::Duration(5.0).sleep();
}

bool canPickObject(Object object) {
    double travelTime = (0.780 - pickYPosition - object.y)/conveyorSpeed;
    ros::Duration timeSpend = ros::Time::now() - object.stamp;
    ros::Duration timeLeft = ros::Duration(travelTime) - timeSpend;
    if (timeLeft.toSec() > 0.1 && object.x > 0.15) {
        return true;
    } else {
        ROS_INFO("%s object skiped\n", object.type.c_str());
        return false;
    }
}

void pick(std::vector<double> position) {
    std::vector<double> downPick = position;
    downPick[2] = downPick[2] - 0.035;
    std::vector<double> upPick = position;

    go_to_position(downPick);
    go_to_position(upPick);
}

void goToBucket(string objectType) {
    if (objectType == "red") {
        go_to_joint_position(redJointGoal);
    }
    if (objectType == "yellow") {
        go_to_joint_position(yellowJointGoal);
    }
    if (objectType == "blue") {
        go_to_joint_position(blueJointGoal);
    }
}

void tryPickNext() {
    if (isDonePicking && !objects.empty()) {
        isDonePicking = false;
        Object pickObject = objects.front();
        objects.pop();

        ROS_INFO("Picking %s object", pickObject.type.c_str());

        std::vector<double> pickPosition = computePickPosition(pickObject);

        ROS_INFO("x: %f", pickPosition[0]);
        ROS_INFO("y: %f", pickPosition[1]);
        ROS_INFO("z: %f", pickPosition[2]);

        if (canPickObject(pickObject)) {
            go_to_position(pickPosition);
            waitForObject(pickObject);
            set_gripper_state(true);
            pick(pickPosition);
            goToBucket(pickObject.type.c_str());
            set_gripper_state(false);
            go_to_joint_position(defaultJointGoal);

            ROS_INFO("DONE!\n");
        }

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

    moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
    move_group = &group;

    ros::Subscriber subscriber = nodeHandle.subscribe("objects", 1000, objectCallback);

    spinner.start();

    configure_serial_port();

    go_to_joint_position(defaultJointGoal);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        tryPickNext();
        //printPose();
        loop_rate.sleep();
    }
    
    ros::waitForShutdown();

    close(serial_port);

    return 0;
}
