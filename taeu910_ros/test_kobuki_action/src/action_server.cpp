#include "ros/ros.h" //
#include "actionlib/server/simple_action_server.h" //

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "test_kobuki_action/Kobuki1Action.h" //

// class Kobuki1Action
class MoveBaseAction
{
    // protected:
    // ros::NodeHandle nh_;
    // actionlib::SimpleActionServer<test_kobuki_action::Kobuki1Action> as_;
    // std::string action_name_;
    // test_kobuki_action::Kobuki1Feedback feedback_;
    // test_kobuki_action::Kobuki1Result result_;
    protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as_;
    std::string action_name_;
    move_base_msgs::MoveBaseFeedback feedback_;
    move_base_msgs::MoveBaseResult result_;
    // actionlib_msgs::GoalStatusArray status_;

    public:
    // Kobuki1Action(std::string name) :
    // as_(nh_, name, boost::bind(&Kobuki1Action::executeCB, this, _1), false),
    MoveBaseAction(std::string name) :
    as_(nh_, name, boost::bind(&MoveBaseAction::executeCB, this, _1), false),
    
    action_name_(name)
    {
        as_.start();
    }

    // ~Kobuki1Action(void)
    ~MoveBaseAction(void)
    {
    }
    void goalCB(){//

    }

    // void executeCB(const test_kobuki_action::Kobuki1GoalConstPtr &goal)
    void executeCB(const move_base_msgs::MoveBaseGoalConstPtr &goal)
    {
        ros::Rate r(1);
        bool success = true;

        // feedback_.sequence.clear();
        // feedback_.sequence.push_back(0);
        // feedback_.sequence.push_back(1);
        
               
        // ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    
        // for(int i=1; i<=goal->order; i++)
        // {
        //     if(as_.isPreemptRequested() || !ros::ok())
        //     {
        //         ROS_INFO("%s: Preempted", action_name_.c_str());

        //         as_.setPreempted();
        //         success = false;
        //         break;
        //     }

        //     feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);

        //     as_.publishFeedback(feedback_);

        //     r.sleep();
        // }

        // if(success)
        // {
        //     result_.sequence = feedback_.sequence;
        //     ROS_INFO("%s: Succeeded", action_name_.c_str());

        //     as_.setSucceeded(result_);
        // }


        // move_base_msgs::MoveBaseFeedback feedback_;
        // double N = 100;
        // double dx = goal->target_pose.pose.position.x/N;
        // double dy = goal->target_pose.pose.position.y/N;
        // fprintf(stderr,"goal is (%f,%f)", goal->target_pose.pose.position.x,goal->target_pose.pose.position.y);
        // // start executing the action
        // for(int i = 1; i <= N; i++) {
        //     // check that preempt has not been requested by the client
        //     if (as_->isPreemptRequested() || !ros::ok()) {
        //         printf("%s: Preempted", action_name_.c_str());
        //         // set the action state to preempted
        //         as_->setPreempted();
        //         success = false;
        //         break;
        //     }

        //     feedback_.base_position.pose.position.x = dx*i;
        //     feedback_.base_position.pose.position.y = dy*i;
        //     fprintf(stderr,"Progress is (%f,%f)", feedback_.base_position.pose.position.x, feedback_.base_position.pose.position.y);

        //     // publish the feedback
        //     as_->publishFeedback(feedback_);
        //     // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        //     r.sleep();
        // }

        // if(success) {
        //  fprintf(stderr,"%s: Succeeded", action_name_.c_str());
        //  // set the action state to succeeded
        //   as_->setSucceeded(result_);
        // }

    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "action_server"); // Initializes Node Name
    // Kobuki1Action fibonacci("test_kobuki_action"); // Kobuki Declaration (Action Name: test_kobuki_action)
    MoveBaseAction fibonacci("move_base_msgs"); // Kobuki Declaration (Action Name: move_base_msgs)
    ros::spin();
    return 0;
}