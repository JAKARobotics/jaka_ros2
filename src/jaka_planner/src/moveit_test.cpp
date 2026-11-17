#include "rclcpp/rclcpp.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/moveit_msgs/msg/display_robot_state.hpp"  
#include "moveit_msgs/moveit_msgs/msg/display_trajectory.hpp"   
#include "moveit_msgs/moveit_msgs/msg/attached_collision_object.hpp"  
#include "moveit_msgs/moveit_msgs/msg/collision_object.hpp"  
#include "moveit_visual_tools/moveit_visual_tools.h"

#include <moveit_msgs/moveit_msgs/msg/joint_limits.hpp>
#include <moveit/robot_state/robot_state.h>
// #include <moveit/robot_state/joint_model_group.h>


#include "std_srvs/srv/empty.hpp"

using namespace std;

void sigintHandler(int /*sig*/) {
    rclcpp::shutdown();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    signal(SIGINT, sigintHandler);
    auto node = rclcpp::Node::make_shared("jaka_planner", options);

    // Declare the parameter for robot model with a default value of "zu3"
    string model = node->declare_parameter<string>("model", "zu3");

    // Automatically construct the PLANNING_GROUP by concatenating "jaka_" + model
    string PLANNING_GROUP = "jaka_" + model;
    RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "Using PLANNING_GROUP: %s", PLANNING_GROUP.c_str());

    // Start a thread on the CPU
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    thread spinner_thread([&executor]() {
        executor.spin();
    });
    
    //  MoveIt uses a JointModelGroup to store the joints of the robot arm, called PLANNING_GROUP. 
    // The "planning group" and the "joint model group" can be used interchangeably throughout the movement.
    // The official configuration file defines it as two move_groups: {manipulator, endeffector}
    // static const string PLANNING_GROUP = "jaka_zu3";
    // By creating an instance of the planning_interface:`MoveGroupInterface` class, you can easily connect, control, or plan a planning group
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    // Add or remove obstacles in the virtual world by using the planning_scene_interface: `PlanningSceneInterface` class
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // Get the state of the robotic arm
    const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    
    rclcpp::Duration du_1(5, 0);

    //random pose
    // move_group.setRandomTarget();

    //Cartesian spatial goal point planning
    // geometry_msgs::Pose target_pose1;
    // target_pose1.position.x = 0.196444;
    // target_pose1.position.y = -0.391179;
    // target_pose1.position.z = 0.382422;
    // target_pose1.orientation.w = 0.029870;
    // target_pose1.orientation.x = 0.458695;
    // target_pose1.orientation.y = 0.887736;
    // target_pose1.orientation.z = -0.025148;
    // move_group.setPoseTarget(target_pose1);

    //Define a Plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //Use a Boolean variable to mark whether the motion planning is successful
    bool success =(move_group.plan(my_plan)== moveit::core::MoveItErrorCode::SUCCESS);
    // ROS_INFO("Visualizing plan 1 (cartesian space goal) %s",success?"":"FAILED");
    // ROS_INFO("OK");

    // move_group.move();   //The move() function is used to perform motion planning and execution operations, and parameters cannot be passed
    // move_group.execute(my_plan); //The execute() function will block the program
    // move_group.asyncExecute(my_plan);  //asynchronous execution
    // ros::Duration(2).sleep();
    // move_group.stop();


    // Motion planning in joint space. This will replace the pose target we set above.
    //First, we'll create a pointer that references the current robot state. RobotState is an object containing all current position/velocity/acceleration data.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    // Next get the current set of joint values for the group.
    vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    // move_group.setStartStateToCurrentState();

    for (int i = 0; i < 2; i++)
    {
        // Now, let's modify the joint, plan to the new joint space target and visualize the plan.
        move_group.setStartStateToCurrentState();
        joint_group_positions[0] = 0.0;  // radians
        joint_group_positions[1] = 1.57;  
        joint_group_positions[2] = -1.57;  
        joint_group_positions[3] = 1.57;  
        joint_group_positions[4] = 1.57;  
        joint_group_positions[5] = 0.0;  
        move_group.setJointValueTarget(joint_group_positions);
        success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "Visualizing Plan 1 (joint space goal) success: %s", success ? "True" : "False");
        RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "OK");

        // move_group.execute(my_plan);
        move_group.move();
        rclcpp::sleep_for(chrono::milliseconds(500));

        move_group.setStartStateToCurrentState();
        joint_group_positions[0] = 1.57;  // radians
        joint_group_positions[1] = 1.57;  
        joint_group_positions[2] = -1.57;  
        joint_group_positions[3] = 1.57;  
        joint_group_positions[4] = 1.57;  
        joint_group_positions[5] = 0.0;  
        move_group.setJointValueTarget(joint_group_positions);
        success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "Visualizing Plan 2 (joint space goal) success: %s", success ? "True" : "False");
        RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "OK");

        // move_group.execute(my_plan);
        move_group.move();
        rclcpp::sleep_for(chrono::milliseconds(500));

        joint_group_positions[0] = 1.57;  // radians
        joint_group_positions[1] = 1.57;  
        joint_group_positions[2] = 1.57;  
        joint_group_positions[3] = 1.57;  
        joint_group_positions[4] = 1.57;  
        joint_group_positions[5] = 0.0;  
        move_group.setJointValueTarget(joint_group_positions);
        success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "Visualizing Plan 3 (joint space goal) success: %s", success ? "True" : "False");
        RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "OK");

        // move_group.execute(my_plan);
        move_group.move();
        rclcpp::sleep_for(chrono::milliseconds(500));

    }

    move_group.setStartStateToCurrentState();
    joint_group_positions[0] = 0.0;  // radians
    joint_group_positions[1] = 1.57;  
    joint_group_positions[2] = -1.57;  
    joint_group_positions[3] = 1.57;  
    joint_group_positions[4] = 1.57;  
    joint_group_positions[5] = 0.0;  
    move_group.setJointValueTarget(joint_group_positions);
    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "Visualizing Plan 4 (joint space goal) success: %s", success ? "True" : "False");
    RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "OK");

    // move_group.execute(my_plan);
    move_group.move();
    rclcpp::sleep_for(chrono::seconds(1));
    

    rclcpp::shutdown();

    // Join spinner thread
    if (spinner_thread.joinable()) {
        spinner_thread.join();
    }

    return 0;

}