#include "ros/ros.h" //
#include "actionlib/client/simple_action_client.h" //
#include "actionlib/client/terminal_state.h" //
#include "test_kobuki_action/KobukiMsgAction.h" //

int main (int argc, char **argv)
{
    ros::init(argc, argv, "action_client");
    
    actionlib::SimpleActionClient<test_kobuki_action::KobukiMsgAction> ac("test_kobuki_action", true);

    ROS_INFO("Waiting for cation ser to start.");
    ac.waitForServer();

    ROS_INFO("Action server started, sending goal.");
    test_kobuki_action::KobukiMsgGoal goal;
    goal.order = 1;
    goal.position_x = 4.0;
    goal.position_y = 4.0;
    ac.sendGoal(goal);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if(finished_before_timeout){
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else{
        ROS_INFO("Action did not finish before the time out.");
    }
    return 0;
}
