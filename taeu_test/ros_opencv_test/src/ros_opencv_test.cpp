#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

int main (int argc, char** argv){
	ros::init(argc, argv, "cv_bridge_test");
	
	cv::VideoCapture cap(0);	
	cv::Mat frame;
	
	if(cap.isOpened()){
		while(1){
			cap >> frame;
			
			cv::imshow("streaming video", frame);
			if(cv::waitKey(1) == 27) break;	
		}
	}
	
	else{
		std::cout << "NO FRAME, CHECK YOUR CAMERA!" << std::endl;
	}	
	
	cv::destroyAllWindows();
	
	return 0;
}