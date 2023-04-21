#include "ros/ros.h" //
#include "actionlib/client/simple_action_client.h" //
#include "actionlib/server/simple_action_server.h" //
#include "actionlib/client/terminal_state.h" //
#include "geometry_msgs/PoseWithCovarianceStamped.h"
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
#include <iostream>
#include <cstring>

#include <typeinfo> // 자료형 확인 typeid(a).name()

using namespace std;
using namespace cv;
// #include "nav_msgs/Odometry.h"
#include "darknet_ros_msgs/BoundingBoxes.h"

void personCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& person_) {
    
    cout << person_->bounding_boxes[0].xmin << endl;
    cout << person_->bounding_boxes[0].ymin << endl;
}


int main( int argc, char** argv ){
  ros::init(argc, argv, "tracking");
  ros::NodeHandle nh;
  ros::Subscriber tracking_sub = nh.subscribe("/darknet_ros/bounding_boxes", 100, personCallback);
  ros::spin();


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
