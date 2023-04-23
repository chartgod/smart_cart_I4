#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_cam_publisher");
  ros::NodeHandle nh;
  ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/usb_cam/image_raw", 1);

  cv::VideoCapture cap(0);
  if (!cap.isOpened()) {
    ROS_ERROR("Cannot open camera");
    return -1;
  }

  while (nh.ok()) {
    cv::Mat frame;
    cap >> frame;
    if (frame.empty()) {
      ROS_ERROR("No captured frame");
      break;
    }

    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = ros::Time::now();
    cv_image.header.frame_id = "usb_cam";
    cv_image.encoding = "bgr8";
    cv_image.image = frame;
    sensor_msgs::Image image_msg;
    cv_image.toImageMsg(image_msg);
    image_pub.publish(image_msg);

    ros::spinOnce();
  }

  return 0;
}
