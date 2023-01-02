#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <string>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <intera_core_msgs/IOComponentCommand.h>

namespace my_planning
{
   class MyPlanningClass
   {
   public:
       MyPlanningClass(ros::NodeHandle *nh); //: move_group(PLANNING_GROUP);
       void goToPoseGoal();
       void goToPoseGoal(geometry_msgs::Pose &pose);
       void goToJointState(double *joint_angles);
       void cartesianPath(geometry_msgs::Pose &pose);
       void resetValues();
       void addObjects();
       void makeBox(std::string blk_name, double *pose);
       void removeObjects();
       void printRobotInfo();
       void openGripper();
       void closeGripper();

   private:
       const std::string PLANNING_GROUP = "right_arm";

       moveit::planning_interface::MoveGroupInterface move_group;
       moveit::planning_interface::PlanningSceneInterface virtual_world;
       const robot_state::JointModelGroup *joint_model_group;
       moveit::planning_interface::MoveGroupInterface::Plan my_plan;

       geometry_msgs::Pose target_pose1;
       ros::Publisher gripper_pub;
   };
}



