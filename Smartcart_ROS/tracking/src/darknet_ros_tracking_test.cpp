#include "ros/ros.h" //
#include "actionlib/client/simple_action_client.h" //
#include "actionlib/server/simple_action_server.h" //
#include "actionlib/client/terminal_state.h" //
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseGoal.h"

#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/tracking/tracking.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <cstring>

#include <typeinfo> // 자료형 확인 typeid(a).name()

using namespace std;
using namespace cv;
// #include "nav_msgs/Odometry.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "tracking/TrackingMsg.h"

int z = 10;
Rect2d bbox;// Rect( Point( x1, y1 ), Point( x2, y2) );
bool initTracking = false;
void personCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& person_) {
    if (person_->bounding_boxes[0].probability > 0.8 && z == 0){
      int x1 = 0, x2 = 0, y1 = 0, y2 = 0;
      
      cout << person_->bounding_boxes[0].probability << endl;
      cout << person_->bounding_boxes[0].xmin << endl;
      cout << person_->bounding_boxes[0].ymin << endl;
      cout << person_->bounding_boxes[0].xmax << endl;
      cout << person_->bounding_boxes[0].ymax << endl;
      x1 = person_->bounding_boxes[0].xmin;
      y1 = person_->bounding_boxes[0].ymin;
      x2 = person_->bounding_boxes[0].xmax;
      y2 = person_->bounding_boxes[0].ymax;
      bbox = Rect(Point(x1,y1),Point(x2,y2));
      z = 1;
    }
}

void appCallback(const tracking::TrackingMsg::ConstPtr& app_msg) {
  if (app_msg -> data == 0){
    z = app_msg -> data;
  }
  if(app_msg -> data == 10){
    z = app_msg -> data;
    initTracking = false;
  }
}


int main( int argc, char** argv ){

  ros::init(argc, argv, "tracking");

  ros::NodeHandle nh;
  ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/usb_cam/image_raw", 1);
  ros::Publisher tracking_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  ros::Subscriber person_sub = nh.subscribe("/darknet_ros/bounding_boxes", 100, personCallback);
  ros::Subscriber tracking_msg_sub = nh.subscribe("/tracking/TrackingMsg", 100, appCallback);

  Ptr<Tracker> tracker = TrackerKCF::create();
  cv::VideoCapture cap(0);
  int frame_width = 640 ;
  int frame_height = 480 ;
  cv::Point2d frame_center(frame_width/2, frame_height/2);
  cap.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
  
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
    
    if(z==1){
      tracker->init(frame, bbox);
      initTracking = true;
      z = 2;
    }
    if(initTracking == true){
      tracker->update(frame,bbox);
      // tracker->getBoundingBox();
      cv::Point2d center(bbox.x + bbox.width / 2.0, bbox.y + bbox.height / 2.0);
      cv::circle(frame, center, 5, cv::Scalar(0, 255, 0), 2);
      rectangle( frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );
      geometry_msgs::Twist goal_;
      // angular
      if (frame_center.x > center.x ){
        goal_.angular.z = 0.5;
      }
      // else if (frame_center.x < center.x ){
      //   goal_.angular.z = -0.5;
      // }
      // linear
      if (frame_center.y > center.y ) {
        goal_.linear.x = 0.5;
      }
      else if (frame_center.y < center.y ){
        goal_.linear.x = -0.5;
      }
      
      
      tracking_pub.publish(goal_);
    }
  
    imshow("tracker",frame);
    if(waitKey(1)==27)break;            
         
    

    ros::spinOnce();
  }

  //quit on ESC button
  
  // ros::NodeHandle nh;
  // ros::Subscriber tracking_sub = nh.subscribe("/darknet_ros/bounding_boxes", 100, personCallback);
  // ros::spin();


  // show help
//   if(argc<2){
//     cout<<
//       " Usage: tracker <video_name>\n"
//       " examples:\n"
//       " example_tracking_kcf Bolt/img/%04d.jpg\n"
//       " example_tracking_kcf faceocc2.webm\n"
//       << endl;
//     return 0;
//   }




//   // declares all required variables
//   Rect2d roi;
//   Mat frame;
// //   create a tracker object
// //   cv::Ptr<cv::TrackerKCF> tracker; 
// //   tracker = cv::TrackerKCF::create();
//   Ptr<Tracker> tracker = TrackerKCF::create();
//   // set input video
//   cv::VideoCapture cap(0);
//   cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
//   cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
// //   std::string video = argv[1];
// //   VideoCapture cap(video);
//   // get bounding box
//   cap >> frame;
//   // roi=selectROI("tracker",frame);
//   int x1 = 250, x2 = 300, y1 = 300, y2 = 350;
//   roi = Rect(Point(x1,y1),Point(x2,y2));  // frame( Rect( Point( x1, y1 ), Point( x2, y2) ) )
//   cout << "type(roi): " << typeid(roi).name() << endl;
//   cout << "roi: " << roi << endl;
//   //quit if ROI was not selected
//   if(roi.width==0 || roi.height==0)
//     return 0;
//   // initialize the tracker
//   tracker->init(frame,roi);
//   // perform the tracking process
//   printf("Start the tracking process, press ESC to quit.\n");
//   for ( ;; ){
//     // get frame from the video
//     cap >> frame;
//     // stop the program if no more images
//     if(frame.rows==0 || frame.cols==0)
//       break;
//     // update the tracking result
//     tracker->update(frame,roi);
//     // draw the tracked object
//     rectangle( frame, roi, Scalar( 255, 0, 0 ), 2, 1 );
//     // show image with the tracked object
//     imshow("tracker",frame);
//     //quit on ESC button
//     if(waitKey(1)==27)break;
//   }
  return 0;
}
