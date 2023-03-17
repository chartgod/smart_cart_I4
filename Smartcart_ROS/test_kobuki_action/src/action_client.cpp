#include "ros/ros.h" //
#include "actionlib/client/simple_action_client.h" //
#include "actionlib/client/terminal_state.h" //
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseGoal.h"

#include "test_kobuki_action/Kobuki1Action.h" //
	

int main (int argc, char **argv)
{
    // ros::init(argc, argv, "action_client");
    // actionlib::SimpleActionClient<test_kobuki_action::Kobuki1Action> ac("test_kobuki_action", true);
    // ROS_INFO("Waiting for action server to start.");
    // ac.waitForServer();
    // ROS_INFO("Action server started, sending goal.");
    // test_kobuki_action::Kobuki1Goal goal;
    // goal.order = 20;
    // ac.sendGoal(goal);
    // bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
    
    ros::init(argc, argv, "action_client");
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.seq = 0;
    goal.target_pose.header.stamp.sec = 0;
    goal.target_pose.header.stamp.nsec = 0;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = 2.5;
    goal.target_pose.pose.position.y = 2.5;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.025547;
    goal.target_pose.pose.orientation.w = 0.98381429;

    // move_base_msgs::MoveBaseActionGoal goal;
    // // actionlib_msgs::GoalID cancel;
    // goal.goal.target_pose.header.seq = 0;
    // goal.goal.target_pose.header.stamp.sec = 0;
    // goal.goal.target_pose.header.stamp.nsec = 0;
    // goal.goal.target_pose.header.frame_id = "map";
    // goal.goal.target_pose.pose.position.x = 4.0;
    // goal.goal.target_pose.pose.position.y = 4.0;
    // goal.goal.target_pose.pose.position.z = 0.0;
    // goal.goal.target_pose.pose.orientation.x = 0.0;
    // goal.goal.target_pose.pose.orientation.y = 0.0;
    // goal.goal.target_pose.pose.orientation.z = 0.025547;
    // goal.goal.target_pose.pose.orientation.w = 0.98381429;
    ac.sendGoal(goal);
        
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if(finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    return 0;
}