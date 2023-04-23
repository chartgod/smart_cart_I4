#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/usb_cam/image_raw", 1);

    cv::VideoCapture cap(0);
    if(!cap.isOpened()) {
        ROS_ERROR("Failed to open camera.");
        return -1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    ros::Rate loop_rate(30); // set publishing rate to 30 Hz

    while (nh.ok()) {
        cv::Mat frame;
        cap >> frame;

        // Preprocess the image to increase fps
        cv::Mat resized;
        cv::resize(frame, resized, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);

        // Convert the image to ROS message and publish it
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized).toImageMsg();
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    cap.release();

    return 0;
}
