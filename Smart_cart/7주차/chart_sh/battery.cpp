#include <ros/ros.h>
#include <kobuki_msgs/SensorState.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

int img_height = 100;
int img_width = 300;
cv::Mat img(img_height, img_width, CV_8UC3, cv::Scalar(0, 0, 0));

void battery_callback(const kobuki_msgs::SensorState::ConstPtr &msg)
{
    int battery_percent = static_cast<int>(msg->battery * 10 / 16);
    int battery_width = static_cast<int>(battery_percent / 100.0 * (img_width - 20));
    cv::rectangle(img, cv::Point(10, 10), cv::Point(battery_width + 10, img_height - 10), cv::Scalar(0, 255, 0), -1);
    cv::putText(img, "Battery: " + std::to_string(battery_percent) + "%", cv::Point(img_width / 2 - 50, img_height / 2), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);

    if (msg->battery < 140)
    {
        ROS_WARN("Now I4 battery status low: %d", msg->battery);
    }
    if (msg->battery < 132)
    {
        move_base_msgs::MoveBaseActionGoal action_msg;
        action_msg.goal.target_pose.header.frame_id = "map";
        action_msg.goal.target_pose.header.stamp = ros::Time::now();
        action_msg.goal.target_pose.pose.position.x = 0.0;
        action_msg.goal.target_pose.pose.position.y = 0.0;
        action_msg.goal.target_pose.pose.orientation.w = 1.0;
        action_publisher.publish(action_msg);
    }
    else
    {
        ROS_INFO("Now I4 battery status low: %d", msg->battery);
    }

    cv::imshow("Battery Status", img);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "battery_monitor");
    ros::NodeHandle nh;

    ros::Subscriber battery_sub = nh.subscribe("/mobile_base/sensors/core", 10, battery_callback);
    ros::Publisher action_publisher = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);

    ros::spin();

    return 0;
}
'''CMakeList.txt추가
find_package(OpenCV REQUIRED)
find_package(move_base_msgs REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${move_base_msgs_INCLUDE_DIRS})

add_executable(battery_monitor_cpp src/battery_monitor.cpp)
target_link_libraries(battery_monitor_cpp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${move_base_msgs_LIBRARIES})

package.xml 
<depend>move_base_msgs</depend>
<depend>opencv'''

