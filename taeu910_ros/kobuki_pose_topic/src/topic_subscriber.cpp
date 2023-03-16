#include "ros/ros.h"
//#include "testbot_pose_topic/MsgTutorial.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "topic_publisher");
	ros::NodeHandle nh;

	// ros::Publisher ros_tutorial_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 100);//중요 토픽명 /initialpose바꿈//ros_tutorial_msg", 100);//initialpose
	// ros::Publisher ros_tutorial_pub2 = nh.advertise<move_base_msgs::MoveBaseGoal>("/move_base/goal", 100);
	ros::Publisher ros_tutorial_pub2 = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 100);
	//ros::Publisher ros_tutorial_pub = nh.advertise<testbot_pose_topic::MsgTutorial>("ros_tutorial_msg", 100);

	ros::Rate loop_rate(10);

	// geometry_msgs::PoseWithCovarianceStamped msg;
	move_base_msgs::MoveBaseGoal msg2;

	//int count = 0;

	while(ros::ok())
	{
		// msg.header.seq= 0;
		// msg.header.stamp.sec= 0;
		// msg.header.stamp.nsec= 0;
		// msg.header.frame_id= "map";// = {seq: 0, stamp: {secs: 0, nsecs: 0 }, frame_id: 'map'};
		// msg.pose.pose.position.x= 1.061060;
		// msg.pose.pose.position.y= 17.007341;
		// msg.pose.pose.position.z= -0.001009;
		// msg.pose.pose.orientation.x= -0.000001;
		// msg.pose.pose.orientation.y= 0.003188;
		// msg.pose.pose.orientation.z= 0.025547;
		// msg.pose.pose.orientation.w= 0.98381429;

		// ros_tutorial_pub.publish(msg);

		// loop_rate.sleep();

		msg2.target_pose.header.seq = 0;
		msg2.target_pose.header.stamp.sec = 0;
		msg2.target_pose.header.stamp.nsec = 0;
		msg2.target_pose.header.frame_id = 'map';
		msg2.target_pose.pose.position.x = 2.5;
		msg2.target_pose.pose.position.y = 2.0;
		msg2.target_pose.pose.position.z = 0.0;
		msg2.target_pose.pose.orientation.x = 0.0;
		msg2.target_pose.pose.orientation.y = 0.0;
		msg2.target_pose.pose.orientation.z = 0.0;
		msg2.target_pose.pose.orientation.w = 1.0;

		ros_tutorial_pub2.publish(msg2);

		// msg.stamp = ros::Time::now();
		// msg.data = count;

		// ROS_INFO("send msg = %d", msg.stamp.sec);
		// ROS_INFO("send msg = %d", msg.stamp.nsec);
		// ROS_INFO("send msg = %d", msg.data);

		loop_rate.sleep();

		//++count;
	}

	return 0;
}



// #include "ros/ros.h"
// //#include "testbot_pose_topic/MsgTutorial.h"
// #include "geometry_msgs/PoseWithCovarianceStamped.h"
// #include "geometry_msgs/PoseArray.h"
// #include "geometry_msgs/Pose.h"
// #include "geometry_msgs/PoseStamped.h"

// void msgCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
// {
// 	//handleInitialPoseMessage(*msg);
// 	//amcl
// 	// ROS_INFO("recieve msg = %d", msg -> stamp.sec);
// 	// ROS_INFO("recieve msg = %d", msg -> stamp.nsec);
// 	// ROS_INFO("recieve msg = %d", msg -> data);
// }

// int main(int argc, char **argv)
// {
// 	ros::init(argc, argv, "ros_topic_sub");

// 	ros::NodeHandle nh;

// 	ros::Subscriber ros_tutorial_sub = nh.subscribe("ros_tutorial_msg", 100, msgCallback);

// 	ros::spin();

// 	return 0;
// }


// // void handleInitialPoseMessage(const geometry_msgs::PoseWithCovarianceStamped& msg)
// // {
// //   boost::recursive_mutex::scoped_lock prl(configuration_mutex_);
// //   if(msg.header.frame_id == "")
// //   {
// //     // This should be removed at some point
// //     ROS_WARN("Received initial pose with empty frame_id.  You should always supply a frame_id.");
// //   }
// //   // We only accept initial pose estimates in the global frame, #5148.
// //   else if(stripSlash(msg.header.frame_id) != global_frame_id_)
// //   {
// //     ROS_WARN("Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
// //              stripSlash(msg.header.frame_id).c_str(),
// //              global_frame_id_.c_str());
// //     return;
// //   }

// //   // In case the client sent us a pose estimate in the past, integrate the
// //   // intervening odometric change.
// //   geometry_msgs::TransformStamped tx_odom;
// //   try
// //   {
// //     // wait a little for the latest tf to become available
// //     tx_odom = tf_->lookupTransform(base_frame_id_, msg.header.stamp,
// //                                    base_frame_id_, ros::Time::now(),
// //                                    odom_frame_id_, ros::Duration(0.5));
// //   }
// //   catch(const tf2::TransformException& e)
// //   {
// //     // If we've never sent a transform, then this is normal, because the
// //     // global_frame_id_ frame doesn't exist.  We only care about in-time
// //     // transformation for on-the-move pose-setting, so ignoring this
// //     // startup condition doesn't really cost us anything.
// //     if(sent_first_transform_)
// //       ROS_WARN("Failed to transform initial pose in time (%s)", e.what());
// //     tf2::convert(tf2::Transform::getIdentity(), tx_odom.transform);
// //   }

// //   tf2::Transform tx_odom_tf2;
// //   tf2::convert(tx_odom.transform, tx_odom_tf2);
// //   tf2::Transform pose_old, pose_new;
// //   tf2::convert(msg.pose.pose, pose_old);
// //   pose_new = pose_old * tx_odom_tf2;

// //   // Transform into the global frame

// //   ROS_INFO("Setting pose (%.6f): %.3f %.3f %.3f",
// //            ros::Time::now().toSec(),
// //            pose_new.getOrigin().x(),
// //            pose_new.getOrigin().y(),
// //            tf2::getYaw(pose_new.getRotation()));
// //   // Re-initialize the filter
// //   pf_vector_t pf_init_pose_mean = pf_vector_zero();
// //   pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
// //   pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
// //   pf_init_pose_mean.v[2] = tf2::getYaw(pose_new.getRotation());
// //   pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
// //   // Copy in the covariance, converting from 6-D to 3-D
// //   for(int i=0; i<2; i++)
// //   {
// //     for(int j=0; j<2; j++)
// //     {
// //       pf_init_pose_cov.m[i][j] = msg.pose.covariance[6*i+j];
// //     }
// //   }
// //   pf_init_pose_cov.m[2][2] = msg.pose.covariance[6*5+5];

// //   delete initial_pose_hyp_;
// //   initial_pose_hyp_ = new amcl_hyp_t();
// //   initial_pose_hyp_->pf_pose_mean = pf_init_pose_mean;
// //   initial_pose_hyp_->pf_pose_cov = pf_init_pose_cov;
// //   applyInitialPose();
// // }
