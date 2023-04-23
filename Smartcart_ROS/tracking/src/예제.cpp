#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// 전역 변수
Rect2d bbox;  // 추적할 객체의 bounding box
bool initTracking = false;  // 객체 추적 초기화 여부
Ptr<Tracker> tracker;  // OpenCV KCF 추적기

// 이미지 콜백 함수
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // 이미지 메시지를 OpenCV Mat 형식으로 변환
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    Mat frame = cv_ptr->image;

    // 추적할 객체의 bounding box를 설정하고 추적기 초기화
    if (!initTracking) {
        ROS_INFO("Please select a person to track.");
        imshow("Tracking", frame);
        bbox = selectROI("Tracking", frame, false);
        if (bbox.width == 0 || bbox.height == 0) {
            return;
        }
        tracker = TrackerKCF::create();
        tracker->init(frame, bbox);
        initTracking = true;
    }

    // darknet_ros로부터 받은 객체 검출 결과 가져오기
    // 여기서는 'person' 클래스에 대한 bounding box만 가져오도록 설정했습니다.
    darknet_ros_msgs::BoundingBoxes bbox_msg;
    bbox_msg = ros::topic::waitForMessage<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", ros::Duration(1));
    if (bbox_msg.bounding_boxes.empty()) {
        ROS_INFO("No person detected.");
        return;
    }
    for (int i = 0; i < bbox_msg.bounding_boxes.size(); i++) {
        if (bbox_msg.bounding_boxes[i].Class == "person") {
            Rect2d detection_bbox;
            detection_bbox.x = bbox_msg.bounding_boxes[i].xmin;
            detection_bbox.y = bbox_msg.bounding_boxes[i].ymin;
            detection_bbox.width = bbox_msg.bounding_boxes[i].xmax - bbox_msg.bounding_boxes[i].xmin;
            detection_bbox.height = bbox_msg.bounding_boxes[i].ymax - bbox_msg.bounding_boxes[i].ymin;

            // 추적 성공 시 bounding box를 초록색으로 표시
            if (tracker->update(frame, bbox)) {
                rectangle(frame, bbox, Scalar(0, 255, 0), 2, 1);
            }
            // 추적 실패 시 검출 결과 bounding box를 빨간색으로 표시
            else {
                rectangle(frame, detection_bbox, Scalar(0, 0, 255), 2, 1);
                ROS_INFO("The tracking has failed...");
                tracker->init(frame, detection_bbox);
                bbox = detection_bbox;
            }
            break;
        }
    }

    // 추
