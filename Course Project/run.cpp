#include <planning.h>

int main(int argc, char **argv)
{
   ros::init(argc, argv, "custom_interfacing");
   ros::NodeHandle node_handle;
   ros::AsyncSpinner spinner(2);
   spinner.start();

   if (argc != 2)
   {
       ROS_INFO(" ");
       ROS_INFO("\tUsage:");
       ROS_INFO(" ");
       ROS_INFO("\trosrun planning run  n");
       return 1;
   }

   my_planning::MyPlanningClass my_planning_(&node_handle);
  
   double targ0[7] = {0, -1, 0,0,0,0,1.57};
   double targ1d[7] = {-0.44566,     -0.44555,    -0.70971,     1.1435,    -0.66825,      -1.213,     -0.4943};
   double targ2d[7] = {0.16623,    -0.065923,    -0.89485,    0.19237,    -0.72972,     0.80275,    0.076115};
   double targ3d[7] = {0.33374,     -0.45917,    -0.69873,     1.0683,    -0.60264,    -0.97074,    -0.48948};
   double targ4d[7] = {1.1059,     -0.16078,    -0.94774,     1.0517,     -1.5779 ,    0.48174,      1.1384};

   double targ1u[7] = {-0.36642,     -0.59815,    -0.74016,      0.99382,    -0.78784,     -1.0055,    -0.28228};
   double targ2u[7] = {0.016076,    0.165923,    -0.84517,    -0.053642,    -0.60174,     0.84959,    -0.17939};
   double targ3u[7] = {0.36526,     -0.55917,    -0.83871,       0.8286,    -0.66369,    -0.82339,    -0.22415};
   double targ4u[7] = {1.0891,     -0.32987,     -1.1957,        1.015,      -1.262,     0.23996,     0.87751};

   int selection = atoi(argv[1]);
   switch (selection)
   {
       my_planning_.resetValues();
       break;
   case 1:
       my_planning_.goToPoseGoal();
       break;
   case 20:
       my_planning_.goToJointState(targ0);
       break;
   case 211:
       my_planning_.goToJointState(targ1u);
       my_planning_.openGripper();
       break;
   case 212:
       my_planning_.goToJointState(targ1d);
       my_planning_.closeGripper();
       break;
   case 221:
       my_planning_.goToJointState(targ2u);
       break;
   case 222:
       my_planning_.goToJointState(targ2d);
       break;
   case 231:
       my_planning_.goToJointState(targ3u);
       break;
   case 232:
       my_planning_.goToJointState(targ3d);
       break;
   case 241:
       my_planning_.goToJointState(targ4u);
       break;
   case 242:
       my_planning_.goToJointState(targ4d);
       break;
   case 3:
       //my_planning_.cartesianPath();
       break;
   case 4:
       my_planning_.addObjects();
       break;
   case 5:
       my_planning_.removeObjects();
       break;
   case 6:
       my_planning_.printRobotInfo();
       break;
   case 7:
       my_planning_.openGripper();
       break;
   case 8:
       my_planning_.closeGripper();
       break;
   }

   spinner.stop();
   return 0;
}



