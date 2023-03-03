/** @package exp_assignment3
*
* \file move_arm.cpp
* \brief this node implements the configuration poses of the robot.
*
* \author Ilenia D'Agelo
* \version 1.0
* \date 27/02/2023
*
* \details
*
* Subscribes to: <BR>
*     None
*
* Publishes to: <BR>
*     None
*
* Serivces: <BR>
*    myarm_pose
*
* Client Services: <BR>
*     None
*
* Action Client: <BR>
*     None
*
* Description: <BR>
* In this node are implemented the possible poses of the robot during the game, they
* were previously implemeted thanks to the moveit setup assistant. The service myarm_pose
* allows the choose of the pose by the client that sends a string as request.
*/

#include <ros/ros.h>
//MOVE IT
#include <moveit/move_group_interface/move_group_interface.h>

#include <exp_assignment3/MoveArm.h>

/**
* \brief Callback function of the myarm_pose service.
* \param req, MoveArmRequest
* \param res, MoveArmResponse
* \return true
*
* This function is the callback function of the myarm_pose service the robot can be
* in the zero pose or in the investigation pose.
* This poses can be selected by the client.
*/

bool GoToPose (exp_assignment3::MoveArm::Request &req, exp_assignment3::MoveArm::Response &res) {


    moveit::planning_interface::MoveGroupInterface group("my_arm");
    group.setEndEffectorLink("cluedo_link");
    group.setPoseReferenceFrame("base_link");
    group.setPlannerId("RRTstar");
    group.setNumPlanningAttempts(10);
    group.setPlanningTime(10.0);
    group.allowReplanning(true);
    group.setGoalJointTolerance(0.0001);
    group.setGoalPositionTolerance(0.0001);
    group.setGoalOrientationTolerance(0.001);

    if (req.mypose == "zero") 
    {
        group.setNamedTarget("zero");
        group.move();  
    }

    else if (req.mypose == "investigation")
    {
        group.setNamedTarget("investigation");
        group.move();  
        
    }
    else {
        return false;
    }

    return true;
}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "move_myarm");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService ("myarm_pose", GoToPose);

	ros::AsyncSpinner spinner(1000);
	spinner.start();
	ros::waitForShutdown();

	return 0;
}
