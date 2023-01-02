// rotate_turtlebot.cpp
// Fred Livingston (fjliving@ncsu.edu) 02-02-2022

#include <ros/ros.h>
#include <math.h>
#include <Eigen/Dense>
#include <tf/tf.h>
#include <tf2_eigen/tf2_eigen.h>
#include <std_srvs/Empty.h>
#include "gazebo_msgs/SetModelState.h"
#include <string>

#define degToRad(angleInDegrees) ((angleInDegrees)*M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians)*180.0 / M_PI)

using namespace std;

int main(int argc, char **argv)
{
    // ROS Client
    ros::init(argc, argv, "module1_assignment1");

    // ROS Service-Cient /gazebo/set_model_state
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    float x = 1.0;
    float y = 0.0;
    float z = 0.0;

    // Set Object Position
    geometry_msgs::Point table_position;
    table_position.x = x;
    table_position.y = y;
    table_position.z = z;

    float theta_rads = degToRad(90);
    // Set Object Orientation
    geometry_msgs::Quaternion table_orientation;
    table_orientation.x = 0.0;
    table_orientation.y = 0.0;
    table_orientation.z = sin(theta_rads / 2.0);
    table_orientation.w = cos(theta_rads / 2.0);

    // Object Pose (Position + Orientation)
    geometry_msgs::Pose table_pose;
    table_pose.position = table_position;
    table_pose.orientation = table_orientation;

    // ModelState
    gazebo_msgs::ModelState table_modelstate;
    table_modelstate.model_name = (std::string) "table";
    table_modelstate.pose = table_pose;

    gazebo_msgs::SetModelState srv;
    srv.request.model_state = table_modelstate;
    client.call(srv);

    z = 1.05;
    float rad = 0.15;
    int angle = 0;
    theta_rads = degToRad(angle);
    float psi = degToRad(90);
    // ModelState
    for (int i = 1; i <= 12; i++)
    {
        x = 1 + rad * cos(theta_rads) - rad * sin(theta_rads);
        y = rad * sin(theta_rads) + rad * cos(theta_rads);

        Eigen::Affine3d Tobj = Eigen::Affine3d::Identity();
        Tobj.translation() << x, y, z;
        geometry_msgs::Pose can_pose = tf2::toMsg(Tobj);

        gazebo_msgs::ModelState can_modelstate;
        can_modelstate.model_name = "coke_can" + to_string(i);
        can_modelstate.pose = can_pose;

        srv.request.model_state = can_modelstate;
        client.call(srv);
        ros::Duration(0.1).sleep();

        Tobj.rotate(Eigen::AngleAxisd(theta_rads + degToRad(45), Eigen::Vector3d::UnitZ()));
        can_pose = tf2::toMsg(Tobj);
        can_modelstate.pose = can_pose;

        srv.request.model_state = can_modelstate;
        client.call(srv);
        ros::Duration(0.1).sleep();

        Tobj.rotate(Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitY()));
        can_pose = tf2::toMsg(Tobj);
        can_modelstate.pose = can_pose;

        srv.request.model_state = can_modelstate;
        client.call(srv);
        ros::Duration(0.1).sleep();

        angle = angle + 360 / 12;
        theta_rads = degToRad(angle);
    }

    return 0;
}
