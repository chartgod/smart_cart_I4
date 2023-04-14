#include "ros/ros.h" //
#include "actionlib/client/simple_action_client.h" //
#include "actionlib/server/simple_action_server.h" //
#include "actionlib/client/terminal_state.h" //
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseGoal.h"


#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include "geometry_msgs/PolygonStamped.h"
#include "nav_msgs/Odometry.h"
using namespace std;

#include "test_kobuki_action/KobukiMsgAction.h" //

// #include "thread"
// using std::thread;
float data_x, data_y;

// void poseCallback(const geometry_msgs::PolygonStamped::ConstPtr& pose_) {
void poseCallback(const nav_msgs::Odometry::ConstPtr& pose_) {
    float position[4];

    position[0] = pose_->pose.pose.position.x;
    position[1] = pose_->pose.pose.position.y;
    position[2] = pose_->pose.pose.orientation.z;
    position[3] = pose_->pose.pose.orientation.w;
    
    cout << "position[0]=x:"<< position[0] <<endl;
    cout << "position[1]=y:"<< position[1] <<endl;
    cout << "position[2]=z:"<< position[2] <<endl;
    cout << "position[3]=w:"<< position[3] <<endl;

    char img_file[] = "/home/user1/catkin_ws/src/smart_cart_I4/Smartcart_ROS/rosbook_kobuki/kobuki_navigation/maps/map_1/map.pgm";
    char img_file_save[] = "/home/user1/catkin_ws/src/smart_cart_I4/Smartcart_ROS/rosbook_kobuki/kobuki_navigation/maps/map_1/map.jpg";
    // 수치 조정 필요
    int position_x = 180+((int)position[0] * 170 / 8);
    int position_y = 220-((int)position[0] * 170 / 8);
    cv::Mat img = cv::imread(img_file, 1);
    cv::circle(img, cv::Point(position_x,position_y),4,cv::Scalar(0,0,255),1,-1,0);
    cv::imwrite(img_file_save, img);    
}

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

// void msgCallback(const test_kobuki_action::KobukiMsg::ConstPtr& msg) {
//     ROS_INFO("recieve msg = %s", msg->data_msg);
//     ROS_INFO("recieve msg = %f", msg->data_x);
//     ROS_INFO("recieve msg = %f", msg->data_y);
//     data_x = msg->data_x;
//     data_y = msg->data_y;
    
//     if (msg->data_msg == "0") {
//         stop();
//     }
//     if (msg->data_msg == "1") {
//         goal();
//     }
// }

class KobukiMsgAction{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<test_kobuki_action::KobukiMsgAction> as_;
        std::string action_name_;
        test_kobuki_action::KobukiMsgFeedback feedback_;
        test_kobuki_action::KobukiMsgResult result_;
    public:
        // Initialize action server (Node handle, action name, action callback function)
        KobukiMsgAction(std::string name) :
            as_(nh_, name, boost::bind(&KobukiMsgAction::executeCB, this, _1), false),
            action_name_(name)
        {
            as_.start();
        }

        ~KobukiMsgAction(void){}

        void executeCB(const test_kobuki_action::KobukiMsgGoalConstPtr &goal){
            ros::Rate r(1);
            bool success = true;
            feedback_.sequence.clear();
            feedback_.sequence.push_back(0);
            feedback_.sequence.push_back(1);
            
            ROS_INFO("%s,%i,%i,%i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

            // Action contents 피보나치 수열 계산
            // for(int i=1; i<=goal->order; i++){
            //     // Confirm action cancellation from action client
            //     if(as_.isPreemptRequested() || !ros::ok()){
            //         // Notify action cancelation
            //         ROS_INFO("%s: Preempted",action_name_.c_str());
            //         // Action cancellation and consider action as failure and save to variable
            //         as_.setPreempted();
            //         success = false;
            //         break;
            //     }
            //     feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
            //     // ROS_INFO("%s,%i,%i", action_name_.c_str(), goal->order, feedback_.sequence[i+1]);
            //     as_.publishFeedback(feedback_);
            //     r.sleep();
            // }

            if(success){
                result_.sequence = feedback_.sequence;
                ROS_INFO("%s: Succeeded", action_name_.c_str());
                as_.setSucceeded(result_);
            }
        }

};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "action_server");
    KobukiMsgAction kobukimsg("test_kobuki_action");

    // ros::init(argc, argv, "A_B_action_client");
    // test
    // ros::init(argc, argv, "topic_subscriber");
    // ros::NodeHandle nh;
    // ros::Subscriber test_kobuki_sub = nh.subscribe("kobuki_msg", 100, msgCallback);
    // ros::Subscriber test_kobuki_sub = nh.subscribe("/move_base/global_costmap/footprint", 100, poseCallback);
    // ros::Subscriber test_kobuki_sub = nh.subscribe("/odom", 100, poseCallback);
    ros::spin();
    return 0;

    
    
    // thread t1(goal);
    // //thread t2(stop);

    // t1.join();
    // // t2.join();

}
