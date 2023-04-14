// 작성자 이승헌
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

int main()
{
    // Load YOLOv5 model
    String weights = "best.pt";
    String config = "yolov5s.yaml";
    dnn::Net net = dnn::readNet(config, weights);
    //dnn::Net net = dnn::readNetFromDarknet(config, weights);
    net.setPreferableBackend(dnn::DNN_BACKEND_CUDA);
    net.setPreferableTarget(dnn::DNN_TARGET_CUDA);

    // Initialize camera
    VideoCapture cap(0);
    if (!cap.isOpened())
    {
        cout << "Cannot open camera" << endl;
        return -1;
    }

    // Loop through frames
    while (true)
    {
        // Read frame from camera
        Mat frame;
        cap.read(frame);

        // Prepare input blob
        Mat blob = dnn::blobFromImage(frame, 1 / 255.0, Size(frame.cols, frame.rows), Scalar(), true, false);

        //Mat blob = dnn::blobFromImage(frame, 1 / 255.0, Size(640, 640), Scalar(), true, false);

        // Forward pass
        net.setInput(blob);
        vector<Mat> outs;
        net.forward(outs);

        // Postprocessing
        float confThreshold = 0.5;
        vector<int> classIds;
        vector<float> confidences;
        vector<Rect> boxes;
        for (size_t i = 0; i < outs.size(); i++)
        {
            // Extract detections from output blob
            Mat detectionMat = outs[i];
            for (int j = 0; j < detectionMat.rows; j++)
            {
                Mat scores = detectionMat.row(j).colRange(5, detectionMat.cols);
                Point classIdPoint;
                double confidence;
                minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
                if (confidence > confThreshold)
                {
                    int centerX = (int)(detectionMat.at<float>(j, 0) * frame.cols);
                    int centerY = (int)(detectionMat.at<float>(j, 1) * frame.rows);
                    int width = (int)(detectionMat.at<float>(j, 2) * frame.cols);
                    int height = (int)(detectionMat.at<float>(j, 3) * frame.rows);
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;
                    classIds.push_back(classIdPoint.x);
                    confidences.push_back((float)confidence);
                    boxes.push_back(Rect(left, top, width, height));
                }
            }
        }

        // Check if children are too close
        float minDistance = 50; // set minimum distance in pixels
        for (size_t i = 0; i < boxes.size(); i++)
        {
            if (classIds[i] == 0) // 0 is the index of the children class
            {
                Point center = (boxes[i].tl() + boxes[i].br()) / 2;
                double distance = norm(center - Point(frame.cols / 2, frame.rows / 2));
                if (distance < minDistance)
                {
                    cout << "WARNING: Children are too close!" << endl;
                    // Add code to play a warning sound here
                }
            }
        }

        // Show result
        imshow("Result", frame);
        waitKey(1);
    }

    return 0;
}
