#include "ros/ros.h" //
#include "actionlib/client/simple_action_client.h" //
#include "actionlib/server/simple_action_server.h" //
#include "actionlib/client/terminal_state.h" //
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "move_base/"

// #include "geometry_msgs/PoseStamped.h"

#include "test_kobuki_action/KobukiMsgAction.h" //

#include "thread"
using std::thread;
float data_x, data_y;

void stop() {
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    char input;
    ac.waitForResult();
    // bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
    // std::cout << "Stop : Press 'S' : ";
    // std::cin >> input;
    // if (input == 'S'){
    //     ac.cancelGoal();
    // }
    ac.cancelGoal();
}

void goal() {
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");
    move_base_msgs::MoveBaseGoal goal;
    
    // actionlib::SimpleActionClient<geometry_msgs::PoseStamped> ac("/move_base_simple/goal",100);
    // geometry_msgs::PoseStamped msg;
 
    // char input;
    // double x, y;

    // std::cout << "Current goal: A\nPress 'A' to stay at A, or 'B' to go to B or 'C' to go to C or 'D' to go to D: ";
    // std::cin >> input;

    // if (input == 'A') {
    //     x = 6.0;
    //     y = 0.5;
    // }
    // else if (input == 'B'){
    //     x = 6.5;
    //     y = 5.5;
    // }
    // else if (input == 'C'){
    //     x = 2.0;
    //     y = 5.8;
    // }
    // else if (input == 'D'){
    //     x = 0.0;
    //     y = 0.0;
    // }
    
    ROS_INFO("recieve msg = %f", data_x);
    ROS_INFO("recieve msg = %f", data_y);

    // Set new goal as A 
    goal.target_pose.header.seq = 0;
    goal.target_pose.header.stamp.sec = 0;
    goal.target_pose.header.stamp.nsec = 0;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = data_x;
    goal.target_pose.pose.position.y = data_y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.025547;
    goal.target_pose.pose.orientation.w = 0.98381429;       
        
    ac.sendGoal(goal);

    //std::cout << "Stop : Press 'S' : ";
    //std::cin >> input;
    //if (input == 'S'){
    //    ac.cancelGoal();
    //}

    // Wait for the robot to reach the goal or for the goal to be cancelled
    ac.waitForResult();
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if(finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
        ROS_INFO("Succeeded!");
    }
    else {
        ROS_WARN("Faile!");
    }
}

void msgCallback(const test_kobuki_action::KobukiMsg::ConstPtr& msg) {
    ROS_INFO("recieve msg = %s", msg->data_msg);
    ROS_INFO("recieve msg = %f", msg->data_x);
    ROS_INFO("recieve msg = %f", msg->data_y);
    data_x = msg->data_x;
    data_y = msg->data_y;
    
    if (msg->data_msg == "0") {
        stop();
    }
    if (msg->data_msg == "1") {
        goal();
    }
}

void actionMsg(){
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<test_kobuki_action::KobukiMsgAction> as_;
    std::string action_name_;
    test_kobuki_action::KobukiMsgFeedback feedback_;
    test_kobuki_action::KobukiMsgResult result_;
    Movebase
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "A_B_action_client");
    // test
    // ros::init(argc, argv, "topic_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber test_kobuki_sub = nh.subscribe("kobuki_msg", 100, msgCallback);
    ros::spin();
    return 0;

    
    
    // thread t1(goal);
    // //thread t2(stop);

    // t1.join();
    // // t2.join();

}
