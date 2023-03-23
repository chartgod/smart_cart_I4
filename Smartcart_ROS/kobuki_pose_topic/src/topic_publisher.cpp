//navigation pose 위치지정

#include "ros/ros.h"
//#include "testbot_pose_topic/MsgTutorial.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseGoal.h"

// MoveBaseGoal
int main(int argc, char **argv)
{
	ros::init(argc, argv, "topic_publisher");
	ros::NodeHandle nh;

	ros::Publisher geometry_msgs_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 100);//중요 토픽명 /initialpose바꿈//ros_tutorial_msg", 100);//initialpose
	// ros::Publisher ros_tutorial_pub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 100);

	//ros::Publisher ros_tutorial_pub = nh.advertise<testbot_pose_topic::MsgTutorial>("ros_tutorial_msg", 100);

	ros::Rate loop_rate(10);

	geometry_msgs::PoseWithCovarianceStamped msg;
	// move_base_msgs::MoveBaseGoal msg;

	//int count = 0;

	while(ros::ok())
	{
		msg.header.seq= 0;
		msg.header.stamp.sec= 0;
		msg.header.stamp.nsec= 0;
		msg.header.frame_id= "map";// = {seq: 0, stamp: {secs: 0, nsecs: 0 }, frame_id: 'map'};
		msg.pose.pose.position.x= 3.0;
		msg.pose.pose.position.y= 3.0;
		msg.pose.pose.position.z= 0.0;
		msg.pose.pose.orientation.x= 0.0;
		msg.pose.pose.orientation.y= 0.0;
		msg.pose.pose.orientation.z= 0.0;
		msg.pose.pose.orientation.w= 1.0;

		// msg.stamp = ros::Time::now();
		// // msg.data = count;

		// // ROS_INFO("send msg = %d", msg.stamp.sec);
		// // ROS_INFO("send msg = %d", msg.stamp.nsec);
		// // ROS_INFO("send msg = %d", msg.data);

		geometry_msgs_pub.publish(msg);

		loop_rate.sleep();
		
		// msg.target_pose.header.seq = 0;
		// msg.target_pose.header.stamp.sec = 0;
		// msg.target_pose.header.stamp.nsec = 0;
		// msg.target_pose.header.frame_id = 'map';
		// msg.target_pose.pose.position.x = 1.0;
		// msg.target_pose.pose.position.y = 10.0;
		// msg.target_pose.pose.position.z = 0.0;
		// msg.target_pose.pose.orientation.x = -0.000001;
		// msg.target_pose.pose.orientation.y = 0.003188;
		// msg.target_pose.pose.orientation.z = 0.025547;
		// msg.target_pose.pose.orientation.w = 0.98381429;
		
		// // msg.header.frame_id = 'map';
		// // msg.header.seq =  0;
		// // msg.header.stamp.sec = 0;
		// // msg.header.stamp.nsec = 0;
		// // msg.goal_id.id = 'map';
		// // msg.goal_id.stamp.sec = 0;
		// // msg.goal_id.stamp.nsec = 0;
		// // msg.goal.target_pose.header.seq = 0;
		// // msg.goal.target_pose.header.stamp.sec = 0;
		// // msg.goal.target_pose.header.stamp.nsec = 0;
		// // msg.goal.target_pose.header.frame_id = 'map';
		// // msg.goal.target_pose.pose.position.x = 1.0;
		// // msg.goal.target_pose.pose.position.y = 10.0;
		// // msg.goal.target_pose.pose.position.z = 0.0;
		// // msg.goal.target_pose.pose.orientation.x = -0.000001;
		// // msg.goal.target_pose.pose.orientation.y = 0.003188;
		// // msg.goal.target_pose.pose.orientation.z = 0.025547;
		// // msg.goal.target_pose.pose.orientation.w = 0.98381429;

		// ros_tutorial_pub.publish(msg);

		// // msg.stamp = ros::Time::now();
		// // msg.data = count;

		// // ROS_INFO("send msg = %d", msg.stamp.sec);
		// // ROS_INFO("send msg = %d", msg.stamp.nsec);
		// // ROS_INFO("send msg = %d", msg.data);

		// loop_rate.sleep();

		//++count;
	}

	return 0;
}