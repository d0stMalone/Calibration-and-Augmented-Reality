/*
 * This program demonstrates the detection of SURF (Speeded Up Robust Features) keypoints in real-time video using OpenCV.
 * It captures frames from the default camera, detects SURF keypoints, and draws them on the frames.
 * The number of detected keypoints (SURF features) is displayed in the console.
 */

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/ocl.hpp>

using namespace cv;
using namespace cv::xfeatures2d;
using std::cout;
using std::endl;

int main() {
    cv::ocl::setUseOpenCL(false); // Disable OpenCL to ensure compatibility
    // Initialize video capture on default camera
    VideoCapture cap(2);
    if (!cap.isOpened()) {
        cout << "Could not open the camera" << endl;
        return -1;
    }

    // Initialize the SURF detector
    int minHessian = 6000; // Minimum Hessian value for SURF feature detection
    Ptr<SURF> detector = SURF::create(minHessian);

    while (true) {
        Mat frame;
        // Capture frame-by-frame
        cap >> frame;

        if (frame.empty()) {
            cout << "Failed to capture frame from camera" << endl;
            break;
        }

        // Convert to grayscale
        Mat img_gray;
        cvtColor(frame, img_gray, COLOR_BGR2GRAY);

        std::vector<KeyPoint> keypoints;
        Mat descriptors;

        // Detect keypoints
        detector->detectAndCompute(img_gray, noArray(), keypoints, descriptors);

        // Draw keypoints
        Mat img_keypoints;
        drawKeypoints(img_gray, keypoints, img_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT);

        // Count the number of detected keypoints (SURF features)
        int feature_count = keypoints.size();

        // Show detected (drawn) keypoints
        imshow("SURF Keypoints", img_keypoints);

        cout << "Total number of detected SURF features: " << feature_count << endl;

        if (waitKey(5) >= 0) break; // Wait for a keystroke in the window or 30ms timeout
    }

    // When everything done, release the video capture object
    cap.release();

    // Closes all the frames
    destroyAllWindows();

    return 0;
} 