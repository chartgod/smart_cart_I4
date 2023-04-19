#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/UInt8MultiArray.h>

#include "nav_msgs/Odometry.h"
float position[4];
char img_file[] = "/home/user1/catkin_ws/src/smart_cart_I4/Smartcart_ROS/rosbook_kobuki/kobuki_navigation/maps/map_1/map.pgm";
char img_file_save[] = "/home/user1/catkin_ws/src/smart_cart_I4/Smartcart_ROS/test_kobuki_action/map_img/map.jpg";
// char img_file[] = "/home/chart/catkin_ws/src/rosbook_kobuki/kobuki_navigation/maps/map.pgm";
// char img_file_save[] = "/home/chart/catkin_ws/src/rosbook_kobuki/kobuki_navigation/maps/map.jpg";

void map_img() {
    // 수치 조정 필요
    int position_x = 200+((int)position[0] * 170 / 8);
    int position_y = 190-((int)position[1] * 170 / 8);
    cv::Mat img = cv::imread(img_file, 1);
    cv::circle(img, cv::Point(position_x,position_y),4,cv::Scalar(0,0,255),1,-1,0);
    cv::imwrite(img_file_save, img);
}
// void poseCallback(const geometry_msgs::PolygonStamped::ConstPtr& pose_) {
void poseCallback(const nav_msgs::Odometry::ConstPtr& pose_) {


    position[0] = pose_->pose.pose.position.x;
    position[1] = pose_->pose.pose.position.y;
    position[2] = pose_->pose.pose.orientation.z;
    position[3] = pose_->pose.pose.orientation.w;
    
    // cout << "position[0]=x:"<< position[0] <<endl;
    // cout << "position[1]=y:"<< position[1] <<endl;
    // cout << "position[2]=z:"<< position[2] <<endl;
    // cout << "position[3]=w:"<< position[3] <<endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_map_img_pub");
    
    ros::NodeHandle nh1;
    ros::Subscriber test_kobuki_sub = nh1.subscribe("/odom", 100, poseCallback);
    ros::NodeHandle nh2;
    // ros::Publisher pub = nh2.advertise<std_msgs::UInt8MultiArray>("camera/image", 1);
    ros::Publisher pub = nh2.advertise<std_msgs::UInt8MultiArray>("map_img/image", 1);

    // cv::VideoCapture cap(0);
    // cv::Mat frame;

    while(nh2.ok())
    {
        map_img();
        // cap >> frame;
        cv::Mat frame = cv::imread(img_file_save, 1);
        if(!frame.empty())
        {

            cv::imshow("frame", frame);

            // Encode, Decode image example            
            std::vector<uchar> encode;
            std::vector<int> encode_param;
            
            encode_param.push_back(CV_IMWRITE_JPEG_QUALITY);
            encode_param.push_back(20);
            
            cv::imencode(".jpg", frame, encode, encode_param);
            cv::Mat decode = cv::imdecode(encode, 1);
            cv::imshow("decode", decode);

            // Convert encoded image to ROS std_msgs format
            std_msgs::UInt8MultiArray msgArray;
            msgArray.data.clear();
            msgArray.data.resize(encode.size());
            std::copy(encode.begin(), encode.end(), msgArray.data.begin());

            // Publish msg
            pub.publish(msgArray);

            cv::waitKey(1);
          
        }

        ros::spinOnce();
    }

    return 0;
    
}