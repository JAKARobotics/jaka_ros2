#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/empty.hpp"
// #include "std_srvs/srv/empty.hpp"
// #include "std_srvs/srv/set_bool.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "jaka_planner/JAKAZuRobot.h"
#include "jaka_planner/jkerr.h"
#include "jaka_planner/jktypes.h"

#include <action_msgs/msg/goal_status_array.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
// #include <trajectory_msgs/msg/joint_trajectory.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

#include <string>
#include <map>
#include <thread>
#include <mutex>
#include <chrono>
#include <csignal>

using namespace std;

JAKAZuRobot robot;
const double PI = 3.1415926;

typedef rclcpp_action::Server<control_msgs::action::FollowJointTrajectory> Server;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub;

// NEW: mutex to protect SDK calls (timer thread + goal thread)
std::mutex robot_mtx;

// Map error codes to messages
map<int, string> mapErr = {
    {2,   "ERR_FUCTION_CALL_ERROR"},
    {-1,  "ERR_INVALID_HANDLER"},
    {-2,  "ERR_INVALID_PARAMETER"},
    {-3,  "ERR_COMMUNICATION_ERR"},
    {-4,  "ERR_KINE_INVERSE_ERR"},
    {-5,  "ERR_EMERGENCY_PRESSED"},
    {-6,  "ERR_NOT_POWERED"},
    {-7,  "ERR_NOT_ENABLED"},
    {-8,  "ERR_DISABLE_SERVOMODE"},
    {-9,  "ERR_NOT_OFF_ENABLE"},
    {-10, "ERR_PROGRAM_IS_RUNNING"},
    {-11, "ERR_CANNOT_OPEN_FILE"},
    {-12, "ERR_MOTION_ABNORMAL"}
};

// Determine if the robot has reached the target position.
bool jointStates(const JointValue &joint_pose, int &err_out)
{
    // RobotStatus robotstatus;
    JointValue joint_position;
    // robot.get_robot_status(&robotstatus);
    {
        std::lock_guard<std::mutex> lk(robot_mtx);
        err_out = robot.get_joint_position(&joint_position);
    }
    if (err_out != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("jointStates"),
                     "get_joint_position failed (%d): %s",
                     err_out, mapErr.count(err_out) ? mapErr[err_out].c_str() : "unknown");
        return false;
    }

    bool joint_state = true;
    for (int i = 0; i < 6; i++)
    {
        // double actual_deg = robotstatus.joint_position[i] * 180.0 / PI;
        double actual_deg = joint_position.jVal[i] * 180.0 / PI;
        double target_deg = joint_pose.jVal[i] * 180.0 / PI;
        // Within +/- 0.2 degrees tolerance
        bool ret = (target_deg - 0.2 < actual_deg) && (actual_deg < target_deg + 0.2);
        joint_state = joint_state && ret;
    }

    RCLCPP_INFO(rclcpp::get_logger("jointStates"), 
                "Whether the robot has reached the target position: %d", joint_state);
    return joint_state;
}

// Handle a new FollowJointTrajectory goal
void goalCb(const shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
    // Enable servo mode
    {
        std::lock_guard<std::mutex> lk(robot_mtx);
        robot.servo_move_enable(true);
    }

    // Retrieve the trajectory from the goal
    auto goal = goal_handle->get_goal();
    const auto &traj = goal->trajectory;
    int point_num = traj.points.size();
    RCLCPP_INFO(rclcpp::get_logger("goalCb"), "number of points: %d", point_num);

    if (point_num == 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("goalCb"), "Trajectory has no points. Aborting goal.");
        goal_handle->abort(make_shared<control_msgs::action::FollowJointTrajectory::Result>());
        {
            std::lock_guard<std::mutex> lk(robot_mtx);
            robot.servo_move_enable(false);
        } 
        return;
    }

    float lastDuration = 0.0;
    JointValue joint_pose;

    // Ensure joint_pose is valid even if point_num == 1
    for (int j = 0; j < 6; j++) {
        joint_pose.jVal[j] = traj.points[point_num - 1].positions[j];
    }

    for (int i = 1; i < point_num; i++)
    {
        // Grab the positions from the i-th trajectory point
        for (int j = 0; j < 6; j++) {
            joint_pose.jVal[j] = traj.points[i].positions[j];
        }

        // Convert time_from_start to a float seconds
        float Duration = static_cast<float>(traj.points[i].time_from_start.sec) +
                         static_cast<float>(traj.points[i].time_from_start.nanosec) * 1e-9f;

        // Calculate time delta relative to previous point
        float dt = Duration - lastDuration;
        lastDuration = Duration;

        // step_num matches old ROS1 logic: step_num = dt / 0.008
        int step_num = static_cast<int>(dt / 0.008f);
        step_num = max(step_num, 1);

        int sdk_res = 0;
        {
            std::lock_guard<std::mutex> lk(robot_mtx);
            sdk_res = robot.servo_j(&joint_pose, MoveMode::ABS, step_num);
        }

        // A) NEW: abort immediately on SDK error (collision/protective stop/etc.)
        if (sdk_res != 0)
        {
            {
                std::lock_guard<std::mutex> lk(robot_mtx);
                robot.motion_abort();
                robot.servo_move_enable(false);
            }

            RCLCPP_ERROR(rclcpp::get_logger("goalCb"),
                         "servo_j failed (%d): %s. Aborting goal.",
                         sdk_res, mapErr.count(sdk_res) ? mapErr[sdk_res].c_str() : "unknown");

            auto result = make_shared<control_msgs::action::FollowJointTrajectory::Result>();
            goal_handle->abort(result);
            return;
        }

        RCLCPP_INFO(rclcpp::get_logger("goalCb"), "The return status of servo_j: %d", sdk_res);
        RCLCPP_INFO(rclcpp::get_logger("goalCb"), 
                    "For point no.: %d, Accepted joint angle: %f %f %f %f %f %f, dt=%f, step_num=%d", i,
                    joint_pose.jVal[0], joint_pose.jVal[1], joint_pose.jVal[2],
                    joint_pose.jVal[3], joint_pose.jVal[4], joint_pose.jVal[5],
                    dt, step_num);

        // // Check if the action was canceled in the middle
        // if (goal_handle->is_canceling())
        // {
        //     // stop motion
        //     robot.motion_abort();
        //     robot.servo_move_enable(false);
        //     RCLCPP_INFO(rclcpp::get_logger("goalCb"), "Servo Mode Disable, motion canceled");
        //     auto result = make_shared<control_msgs::action::FollowJointTrajectory::Result>();
        //     goal_handle->canceled(result);
        //     return;
        // }
    }

    // NEW: timeout wait loop (no infinite hang)
    auto result = make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    auto t0 = std::chrono::steady_clock::now();
    const auto max_wait = std::chrono::seconds(5);

    // Wait until the robot is actually at the final position, or until canceled
    while (rclcpp::ok())
    {
        int read_err = 0;
        if (jointStates(joint_pose, read_err))
        {
            {
                std::lock_guard<std::mutex> lk(robot_mtx);
                robot.servo_move_enable(false);
            }
            RCLCPP_INFO(rclcpp::get_logger("goalCb"), "Servo Mode Disable: Target Reached");
            RCLCPP_INFO(rclcpp::get_logger("goalCb"), 
                        "==============Motion stops or reaches the target position==============");
            goal_handle->succeed(result);
            return;
        }

        // if (goal_handle->is_canceling())
        // {
        //     robot.motion_abort();
        //     robot.servo_move_enable(false);
        //     RCLCPP_INFO(rclcpp::get_logger("goalCb"), "Servo Mode Disable");
        //     RCLCPP_INFO(rclcpp::get_logger("goalCb"), 
        //                 "==============Motion stops or was canceled==============");
        //     auto result = make_shared<control_msgs::action::FollowJointTrajectory::Result>();
        //     goal_handle->canceled(result);
        //     return;
        // }

        // NEW: if we cannot read joint position, abort
        if (read_err != 0)
        {
            {
                std::lock_guard<std::mutex> lk(robot_mtx);
                robot.motion_abort();
                robot.servo_move_enable(false);
            }
            RCLCPP_ERROR(rclcpp::get_logger("goalCb"),
                         "Aborting goal because get_joint_position failed (%d).", read_err);
            goal_handle->abort(result);
            return;
        }

        if (std::chrono::steady_clock::now() - t0 > max_wait)
        {
            {
                std::lock_guard<std::mutex> lk(robot_mtx);
                robot.motion_abort();
                robot.servo_move_enable(false);
            }
            RCLCPP_ERROR(rclcpp::get_logger("goalCb"),
                         "Timeout waiting for final position. Aborting goal.");
            goal_handle->abort(result);
            return;
        }

        rclcpp::sleep_for(chrono::milliseconds(50));
    }

    // // After processing all points, check if the goal was canceled
    // if (goal_handle->is_canceling()) {
    //     robot.motion_abort();
    //     robot.servo_move_enable(false);
    //     RCLCPP_INFO(rclcpp::get_logger("goalCb"), "Servo Mode Disabled, Motion Canceled");
    //     auto result = make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    //     goal_handle->canceled(result);
    //     return;
    // }

    // If rclcpp not ok
    {
        std::lock_guard<std::mutex> lk(robot_mtx);
        robot.motion_abort();
        robot.servo_move_enable(false);
    }
    goal_handle->abort(result);
}

// Publish the robot's joint states to /joint_states (for RViz / MoveIt feedback)
void joint_states_callback(rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr &joint_states_pub)
{
    sensor_msgs::msg::JointState joint_msg;
    // RobotStatus robotstatus;
    JointValue joint_position;
    // robot.get_robot_status(&robotstatus);
    int ret = 0;
    {
        std::lock_guard<std::mutex> lk(robot_mtx);
        ret = robot.get_joint_position(&joint_position);
    }
    if (ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("joint_states_callback"),
                    "get_joint_position failed (%d): %s",
                    ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
        return; // don't publish stale/invalid data
    }

    for (int i = 0; i < 6; i++)
    {
        // joint_msg.position.push_back(robotstatus.joint_position[i]);
        joint_msg.position.push_back(joint_position.jVal[i]);
        joint_msg.name.push_back("joint_" + to_string(i+1));
    }
    joint_msg.header.stamp = rclcpp::Clock().now();
    joint_states_pub->publish(joint_msg);
}

void sigintHandler(int /*sig*/) {
    rclcpp::shutdown();
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    signal(SIGINT, sigintHandler);
    auto node = rclcpp::Node::make_shared("moveit_server");

    // Read parameters
    string default_ip = "10.5.5.100";
    string default_model = "zu3";
    string robot_ip = node->declare_parameter("ip", default_ip);
    string robot_model = node->declare_parameter("model", default_model);

    // rclcpp::Rate rate(125);
    // Connect to robot
    int ret = 0;
    {
        std::lock_guard<std::mutex> lk(robot_mtx);
        ret = robot.login_in(robot_ip.c_str(), false);

        // Turn off servo at startup
        robot.servo_move_enable(false);
    }
    if (ret != 0) {
    RCLCPP_ERROR(node->get_logger(), "login_in failed (%d): %s",
                ret, mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
    return 1;
    }
    rclcpp::sleep_for(chrono::milliseconds(500));

    // Filter param
    {
        std::lock_guard<std::mutex> lk(robot_mtx);
        robot.servo_move_use_joint_LPF(0.5);
    }

    // Power on + enable
    {
        std::lock_guard<std::mutex> lk(robot_mtx);
        ret = robot.power_on();
    }
    if (ret != 0) {
    RCLCPP_ERROR(node->get_logger(), "power_on failed (%d): %s", ret,
                mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
    return 1;
    }
    rclcpp::sleep_for(chrono::seconds(8));

    {
        std::lock_guard<std::mutex> lk(robot_mtx);
        ret = robot.enable_robot();
    }
    if (ret != 0) {
    RCLCPP_ERROR(node->get_logger(), "enable_robot failed (%d): %s", ret,
                mapErr.count(ret) ? mapErr[ret].c_str() : "unknown");
    return 1;
    }
    rclcpp::sleep_for(chrono::seconds(4));

    // Publisher for /joint_states
    joint_states_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // NEW: Publish joint states on a timer (keeps RViz updating during goal execution)
    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(8),   // ~125 Hz
        []() {
            joint_states_callback(joint_states_pub);
        }
    );

    // Create Action Server for FollowJointTrajectory
    auto moveit_server = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
        node,
        "/jaka_" + robot_model + "_controller/follow_joint_trajectory",
        // Goal callback
        [](const rclcpp_action::GoalUUID &uuid,
           shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal) {
            RCLCPP_INFO(rclcpp::get_logger("moveit_server"), "Received goal request");
            (void)uuid; // Avoid unused parameter warning
            (void)goal; // Avoid unused parameter warning
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        // // Cancel callback
        // [](const shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle) {
        //     (void)goal_handle;  // Avoid unused parameter warning
        //     RCLCPP_INFO(rclcpp::get_logger("moveit_server"), "Received cancel request");
        //     return rclcpp_action::CancelResponse::ACCEPT;
        // },
        nullptr,  // Cancel callback removed
        // Execute callback
        [](const shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle) {
            RCLCPP_INFO(rclcpp::get_logger("moveit_server"), "Executing goal");
            // NEW: execute in a separate thread (avoid blocking executor)
            std::thread([goal_handle]() {
                goalCb(goal_handle);
            }).detach();
        }
    );

    RCLCPP_INFO(rclcpp::get_logger("moveit_server"), "==================Moveit Start==================");

    // NEW: proper executor (no manual while loop)
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
