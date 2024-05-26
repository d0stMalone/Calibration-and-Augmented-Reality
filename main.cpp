/* 
* main.cpp
* created by Keval Visaria and Chirag Dhoka Jain
* 
* This file contains all the functions and shows the working of the system on different Key presses.
* This file enables user to calibrate the camera and project 3D objects
* 
*/

#include <opencv2/opencv.hpp>    
#include <opencv2/aruco.hpp>    
#include <opencv2/highgui.hpp>
#include <iostream>
#include "functions.h"
#include "csv_util.h"

using namespace std;
using namespace cv;

Mat originalFrame, detectedFrame, axesFrame, virtualObjectFrame;

Mat distortion_coefficient;

/* Initiazing rotation and translation vectors*/
vector<Mat> rot_vec;
vector<Mat> tran_vec;

Mat PNP_rot_vec;
Mat PNP_tran_vec;

/* Declaring flags used for achieving logic */
bool calibrationFlag = false;
bool positionCalculatedFlag = false;
bool task_4_Flag = false; 
bool task_5_Flag = false;
bool task_6_Flag = false;
bool arucoFlag = false;
bool displayImg = false;
bool extFlag = false;

/* Image counter to store the calibration images */
int imageCounter = 1;

/*Location of the CSV file where we will be storing the camera matrix and distortion coefficient*/
/*Uncomment as per the device used - keval or chirag*/

char CSV[256] = "C:/Users/visar/Desktop/OneDrive - Northeastern University/PRCV/CAR/data.csv"; // Keval
//char CSV[256] = "D:/My source/Spring2024/PRCV/c++/AR/AugmentedReality/data.csv"; //Chirag 

/* Initializing a 3x3 Camera matrix*/
double camera_mat[3][3] = { {1, 0, (double)originalFrame.cols / 2},
                            {0, 1, (double)originalFrame.rows / 2},
                            {0, 0, 1} 
                           };

/* Converting the camera matrix to type CV_64FC1*/
Mat camera_matrix = Mat(3, 3, CV_64FC1, &camera_mat);


int main() {

    /* Checkeredboard pattern size */
    Size patternSize(9, 6); // pattern size of the checkerboard

    vector<Point2f> corner_set;
    vector<Vec3f> point_set;
    vector<vector<Vec3f> > point_list;
    vector<vector<Point2f> > corner_list;

    /* markerIds for aruco markers*/
    vector<int> markerIds;

    /* for saving the markerCorners */
    vector<vector<Point2f>> markerCorners, rejectedCandidates;

    /* Target image that is being displayed on the checkerdboard*/
    Mat targetImage = imread("C:/Users/visar/Desktop/OneDrive - Northeastern University/PRCV/CAR/extenion.jpg");


    // Open the video device
    VideoCapture capdev(0); // Use stack allocation for simplicity
    if (!capdev.isOpened()) {
        cerr << "Unable to open video device\n";
        return -1;
    }
    cout << "Stream Started!\n";

    while (true) {

        capdev >> originalFrame; // Capture frame-by-frame
        if (originalFrame.empty()) {
            cerr << "Frame is empty" << endl;
            break; // Exit if the frame is empty
        }

        //imshow("Video", originalFrame); // Show the frame in the created window


        /* -------------------TASK 1-------------------------------------------------*/

        detectedFrame = originalFrame.clone(); // dectected

        /*
            Takes in the the origial frame received from the camera
            returns the mat/frame with corners detectedFrame.

            The function returns a boolean that keeps on checking if the checkeredboard is found or not

        */
        bool found = getCorners(originalFrame, detectedFrame, patternSize, corner_set);

        /*
            This draws the chessboard corners
        */
        drawChessboardCorners(detectedFrame, patternSize, Mat(corner_set), found);
        imshow("Detect and Extract Chessboard Corners", detectedFrame);

        /* checking if any key is pressed by the user */
        char key = waitKey(25); // Wait for a key press for 25ms

        /* program stops when 'q' or 'Q' or ESC is pressed */
        if (key == 'q' || key == 'Q' || key == 27) {
            break;
        }

        /* -------------------TASK 2-------------------------------------------------*/
            /*
                On press 'S' let the user gets the world and corner coordinates and also the
                RMS re reprojection error for that frame, also save the calibration image.
            */

        else if (key == 's' || key == 'S') {

            /* if the checkerboard is found get the coordinates */
            if (found) {
                getCoordinates(point_set, point_list, corner_set, corner_list);

                /* calibrateCamera function calibrates the camera with all the intrinsic parameters provide to it */
                float rmsReprojectionError = calibrateCamera(point_list, corner_list, originalFrame.size(), camera_matrix, distortion_coefficient, rot_vec, tran_vec, CALIB_FIX_ASPECT_RATIO);

                /*Display the RMS reprojecction error*/
                cout << "RMS re-projection Error: " << rmsReprojectionError << endl;

                /* Saves the image at a location */
                std::string filename = "C:/Users/visar/Desktop/OneDrive - Northeastern University/PRCV/Project1_Images/Calibration_Image_" + std::to_string(imageCounter) + ".jpg";
                cv::imwrite(filename, detectedFrame);
                printf("Calibration Image saved as %s\n", filename.c_str());
                imageCounter++;
            }
            else {
                /* This part executes when there checkerboard is not in frame or is not being detected*/
                cout << "Checkerboard not detectedFrame" << endl;
            }
        }

        /* -------------------TASK 3-------------------------------------------------*/

        /* here we save the final calibration of the camera in a csv file */
        else if (key == 'c' || key == 'C') {

            /* Will only execute if we have saved minimum of 5 calibration images */
            if (corner_list.size() < 5 && found == false) {
                cout << "No enough calibration images input, at least 5 images needed. Current # of images: " << corner_list.size() << endl;
            }
            else {
                cout << "Calibrating ....." << endl;
                cout << "Camera Matrix before Calibration: " << endl << camera_matrix << endl;

                float rmsReprojectionError = calibrateCamera(point_list, corner_list, detectedFrame.size(), camera_matrix, distortion_coefficient, rot_vec, tran_vec, CALIB_FIX_ASPECT_RATIO);

                cout << "Camera Matrix after Calibration: " << endl << camera_matrix << endl;
                cout << "RMS re-projection Error: " << rmsReprojectionError << endl;

                /* Saves the camera matrix and the distortion coefficients in a csv file. */
                append_parameters_csv(CSV, "Camera Matrix", camera_matrix, 0);
                append_parameters_csv(CSV, "Distortion Coefficient", distortion_coefficient, 0);
            }

            calibrationFlag = true;
        }

        /* ------------------ TASK 4 : Calculate Current Position of the Camera ----------------------*/
        /* Here on pressing 'm' or 'M', user will be able to get the Rotation and
            Translation Matrix in real time on the console
        */
        else if (key == 'm' || key == 'M') {

            /* If the checkerboard is detected then only prints the rotation and translation matrix */
            if (found) {
                cout << "Printing Rotation and Translation Matrix: " << endl;
                positionCalculatedFlag = true;
                task_4_Flag = !task_4_Flag;
            }
            else {
                /* This part executes when there checkerboard is not in frame or is not being detected*/
                cout << "Error: Checkerdboard not detectedFrame" << endl;
                task_4_Flag = false;
            }
        }


        /* ------------------ TASK 5: Project Outside Corners or 3D Axes -----------------------------*/
        /* in this part we project the 3D axes on the checkerboard*/
        else if (key == 'a' || key == 'A') {

            /*
                The 3D axes will only be visible when the rotation and translation matrix are being generated
                also when the checkerboard is found.
            */
            if (found && positionCalculatedFlag) {

                cout << "Displaying the 3D Axes: " << endl;
                task_5_Flag = !task_5_Flag;

            }
            /* this exectues when the rotation and translation matrix is not being generated */
            else if (!positionCalculatedFlag && !task_4_Flag) {
                cout << "Error: Positions not generated, please press 'm'" << endl;
            }
            /* This part executes when there checkerboard is not in frame or is not being detected*/
            else if (!found) {
                cout << "Error: Checkerdboard not detectedFrame" << endl;
                task_5_Flag = false;
            }
        }


        /* ------------------- TASK 6: Create a Virtual Object -------------------------------------------------*/
        /* on key press 'v' or 'V' user wil be able to display a virtual object on the checkerboard */
        else if (key == 'v' || key == 'V') {

            /*
                The object will only be visible when the rotation and translation matrix are being generated
                also when the checkerboard is found.
            */
            if (found && positionCalculatedFlag) {
                cout << "Displaying Virtual Object" << endl;
                task_6_Flag = !task_6_Flag;
            }
            /* this exectues when the rotation and translation matrix is not being generated */
            else if (!positionCalculatedFlag && !task_4_Flag && !task_5_Flag) {
                cout << "Error: Positions not generated, please press 'm'" << endl;
            }
            /* This part executes when there checkerboard is not in frame or is not being detected*/
            else if (!found) {
                cout << "Error: Checkerdboard not detectedFrame" << endl;
                task_6_Flag = false;
            }
        }

        /* ------------------- EXTENTION: MULTIPLE TARGETS -------------------------------------------------*/
        /*
            when the user presses 'e' or 'E', he would be able to
            detected mutiple aruco markers with their labels
        */
        else if (key == 'e' || key == 'E') {
            cout << "Detecting Aruco Markers: " << endl;
            arucoFlag = true;
        }
        /* ------------------- EXTENTION: DISPLAYING STATIC IMAGES ON THE CHECKERBOARD -------------------------------------------------*/
        /*
            when the user presses 'd' or 'D', an static image will be displayed
        */
        else if (key == 'd' || key == 'D') {

            if (found) {
                cout << "Displaying image on checkeredboard" << endl;
                displayImg = true;
            }
            /* This part executes when there checkerboard is not in frame or is not being detected*/
            else {
                cout << "Checkerboard not found!" << endl;
            }
        }
        /* ------------------- EXTENTION: DISPLAYING STATIC IMAGES AND VIRTUAL OBJECT ON THE CHECKERBOARD -------------------------------------------------*/
        /*
            when the user presses 't' or 'T', an static image and virtual object will be displayed
        */
        else if (key == 't' || key == 'T') {

            if (found && positionCalculatedFlag && displayImg) {
                cout << "Displaying Virtual Object and image" << endl;
                extFlag = !extFlag;
            }
            /* this exectues when the rotation and translation matrix is not being generated */
            else if (!positionCalculatedFlag && !task_6_Flag && !task_5_Flag && !displayImg) {
                cout << "Error: Positions not generated, please press 'm'" << endl;
            }
            /* This part executes when there checkerboard is not in frame or is not being detected*/
            else if (!found) {
                cout << "Error: Checkerdboard not detectedFrame" << endl;
                task_6_Flag = false;
            }
        }


        /* this part prints the Rotation and Translation Matrix */
        if (task_4_Flag && found) {
            if (calibrationFlag) {

                /* solvePnp function helps in pose estimation*/
                solvePnP(point_set, corner_set, camera_matrix, distortion_coefficient, PNP_rot_vec, PNP_tran_vec);

                cout << endl << "Rotation matrix: " << endl;
                for (int i = 0; i < PNP_rot_vec.rows; i++) {
                    for (int j = 0; j < PNP_rot_vec.cols; j++) {
                        cout << "    " << PNP_rot_vec.at<double>(i, j) << endl;
                    }
                }
                cout << "Translation matrix: " << endl;
                for (int i = 0; i < PNP_tran_vec.rows; i++) {
                    for (int j = 0; j < PNP_tran_vec.cols; j++) {
                        cout << "    " << PNP_tran_vec.at<double>(i, j) << endl;
                    }
                }
            }
        }
        /* This part executes when there checkerboard is not in frame or is not being detected */
        else if (task_4_Flag) {
            cout << "Checkerdboard not detectedFrame" << endl;
        }


        // Check if the task flag, object found flag, and position calculation flag are all true
        if (task_5_Flag && found && positionCalculatedFlag) {

            // Ensure that the number of detected corners is exactly 54, typical for a Rubik's Cube
            if (corner_set.size() != 54) continue;

            // Use solvePnP to estimate the object's pose given object points, image points,
            // the camera matrix, and distortion coefficients
            solvePnP(point_set, corner_set, camera_matrix, distortion_coefficient, PNP_rot_vec, PNP_tran_vec);

            // Define points in the real world to represent the axes
            vector<Vec3f> real_world;
            real_world.push_back(Vec3f(0, 0, 0)); // Origin
            real_world.push_back(Vec3f(0, -3, 0)); // Y-axis point
            real_world.push_back(Vec3f(3, 0, 0));  // X-axis point
            real_world.push_back(Vec3f(0, 0, 3));  // Z-axis point

            // Project these real-world points into the image plane to find their image coordinates
            vector<Point2f> image_points;
            projectPoints(real_world, PNP_rot_vec, PNP_tran_vec, camera_matrix, distortion_coefficient, image_points);

            // Draw lines between the projected points to display the 3D axes on the detected object
            line(detectedFrame, image_points[0], image_points[1], Scalar(0, 0, 255), 2); // Y-axis in red
            line(detectedFrame, image_points[0], image_points[2], Scalar(0, 255, 0), 2); // X-axis in green
            line(detectedFrame, image_points[0], image_points[3], Scalar(255, 0, 0), 2); // Z-axis in blue

            // Show the frame with the drawn 3D axes in a window titled "3D Axes"
            imshow("3D Axes", detectedFrame);
        }



        // Check if the task flag, object found flag, and position calculation flag are all true
        if (task_6_Flag && found && positionCalculatedFlag) {

            // Ensure that the number of detected corners is exactly 54, likely a check for a specific object detection
            if (corner_set.size() != 54) continue;

            // Use solvePnP to estimate the pose of the detected object using point correspondences
            solvePnP(point_set, corner_set, camera_matrix, distortion_coefficient, PNP_rot_vec, PNP_tran_vec);

            vector<Vec3f> house;
            // Base of the house - defining the corners
            house.push_back(Vec3f(-1, 1, 0));  // Bottom-left front
            house.push_back(Vec3f(9, 1, 0));   // Bottom-right front
            house.push_back(Vec3f(-1, -6, 0)); // Top-left front
            house.push_back(Vec3f(9, -6, 0));  // Top-right front

            // Top of the house - defining the corners
            house.push_back(Vec3f(0, 0, 7));  // Bottom-left back
            house.push_back(Vec3f(9, 0, 7));  // Bottom-right back
            house.push_back(Vec3f(0, -6, 7)); // Top-left back
            house.push_back(Vec3f(9, -6, 7)); // Top-right back

            // Top of the cone above the house
            house.push_back(Vec3f(5, -3, 10)); // Apex of the cone

            vector<Point2f> image_points;
            // Project the 3D house points onto the 2D image plane
            projectPoints(house, PNP_rot_vec, PNP_tran_vec, camera_matrix, distortion_coefficient, image_points);

            // Drawing the base and top of the house
            // Connect the dots to draw the base square
            line(detectedFrame, image_points[0], image_points[1], Scalar(0, 0, 255), 5);
            line(detectedFrame, image_points[0], image_points[2], Scalar(0, 0, 255), 5);
            line(detectedFrame, image_points[1], image_points[3], Scalar(0, 0, 255), 5);
            line(detectedFrame, image_points[2], image_points[3], Scalar(0, 0, 255), 5);

            // Drawing the top square
            line(detectedFrame, image_points[4], image_points[5], Scalar(0, 0, 255), 5);
            line(detectedFrame, image_points[4], image_points[6], Scalar(0, 0, 255), 5);
            line(detectedFrame, image_points[5], image_points[7], Scalar(0, 0, 255), 5);
            line(detectedFrame, image_points[6], image_points[7], Scalar(0, 0, 255), 5);

            // Connecting the base and the top
            line(detectedFrame, image_points[0], image_points[4], Scalar(0, 0, 255), 5);
            line(detectedFrame, image_points[1], image_points[5], Scalar(0, 0, 255), 5);
            line(detectedFrame, image_points[2], image_points[6], Scalar(0, 0, 255), 5);
            line(detectedFrame, image_points[3], image_points[7], Scalar(0, 0, 255), 5);

            // Drawing the cone on top
            line(detectedFrame, image_points[4], image_points[8], Scalar(255, 0, 255), 3); // Connect base to apex
            line(detectedFrame, image_points[5], image_points[8], Scalar(255, 0, 255), 3); // Connect base to apex
            line(detectedFrame, image_points[6], image_points[8], Scalar(255, 0, 255), 3); // Connect base to apex
            line(detectedFrame, image_points[7], image_points[8], Scalar(255, 0, 255), 3); // Connect base to apex

            // Show the image with the virtual house drawn on it
            imshow("Virtual Object", detectedFrame);
        }



        // Check if ArUco marker detection is enabled
        if (arucoFlag) {
            // Initialize detector parameters with default values
            aruco::DetectorParameters detectorParams = aruco::DetectorParameters();

            // Get a predefined dictionary of ArUco markers
            aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);

            // Create an ArUco detector instance with the specified dictionary and detector parameters
            aruco::ArucoDetector detector(dictionary, detectorParams);

            // Detect markers in the originalFrame and fill markerCorners and markerIds with the detected markers' information
            detector.detectMarkers(originalFrame, markerCorners, markerIds, rejectedCandidates);

            // Check if any markers were detected
            if (!markerIds.empty()) {
                // Draw3 markers on the originalFrame with their respective IDs
                aruco::drawDetectedMarkers(originalFrame, markerCorners, markerIds);
            }

            // Display the frame with the detected ArUco markers in a window titled "detectedFrame ArUco markers"
            imshow("detectedFrame ArUco markers", originalFrame);
        }


        // Check if image display is enabled
        if (displayImg) {

            // Check if the chessboard has been found in the frame
            if (found) {

                Mat gray;
                // Convert the original frame to grayscale
                cvtColor(originalFrame, gray, COLOR_BGR2GRAY);

                // Define points on the target image corresponding to the corners
                vector<Point2f> targetPoints = {
                    Point2f(0, 0),
                    Point2f((float)targetImage.cols, 0),
                    Point2f((float)targetImage.cols, (float)targetImage.rows),
                    Point2f(0, (float)targetImage.rows)
                };

                // Map the target image to the corners of the detected chessboard in the frame
                vector<Point2f> chessboardPoints = {
                    corner_set[0],
                    corner_set[patternSize.width - 1],
                    corner_set[corner_set.size() - 1],
                    corner_set[corner_set.size() - patternSize.width]
                };

                // Define object points for perspective transformation (assuming a flat surface)
                vector<Point3f> objectPoints = {
                    {-1, 1, 0},
                    {9, 1, 0},
                    {9, -6, 0},
                    {-1, -6, 0}
                };

                // Project the 3D object points to 2D image points
                vector<Point2f> image_points;
                projectPoints(objectPoints, PNP_rot_vec, PNP_tran_vec, camera_matrix, distortion_coefficient, image_points);

                // Calculate the perspective transform matrix from the target image to the image points
                Mat matrix = getPerspectiveTransform(targetPoints, image_points);

                // Warp the target image to fit the perspective of the chessboard's location in the frame
                Mat warpedImage;
                warpPerspective(targetImage, warpedImage, matrix, originalFrame.size());

                // Create a mask for the area of the chessboard
                Mat mask = Mat::zeros(gray.size(), CV_8UC1);
                vector<Point> pts = { image_points[0], image_points[1], image_points[2], image_points[3] };
                fillConvexPoly(mask, pts, Scalar(255), LINE_AA);

                // Create an inverse mask to isolate the chessboard area
                Mat inverseMask;
                bitwise_not(mask, inverseMask);

                // Remove the chessboard area from the original frame
                Mat frameWithoutChessboard;
                originalFrame.copyTo(frameWithoutChessboard, inverseMask);

                // Add the warped target image to the area where the chessboard was removed
                originalFrame = frameWithoutChessboard + warpedImage;

                // Display the frame with the target image overlay
                imshow("Frame with Overlay", originalFrame);
            }
            else {
                // If no chessboard was detected, output an error message
                cout << "Chessboard not detected" << endl;
            }
        }

        if (extFlag) {
            // Check if the chessboard pattern has been successfully detected in the frame.
            if (found) {
                // Convert the original frame to grayscale for processing
                Mat gray;
                cvtColor(originalFrame, gray, COLOR_BGR2GRAY);

                // Perform image overlay if chessboard detected
                vector<Point2f> targetPoints = {
                    Point2f(0, 0),
                    Point2f((float)targetImage.cols, 0),
                    Point2f((float)targetImage.cols, (float)targetImage.rows),
                    Point2f(0, (float)targetImage.rows)
                };
                vector<Point2f> chessboardPoints = {
                    corner_set[0],
                    corner_set[patternSize.width - 1],
                    corner_set[corner_set.size() - 1],
                    corner_set[corner_set.size() - patternSize.width]
                };
                Mat matrix = getPerspectiveTransform(targetPoints, chessboardPoints);
                Mat warpedImage;
                warpPerspective(targetImage, warpedImage, matrix, originalFrame.size());

                // Creating a mask for the area where the chessboard is detected
                Mat mask = Mat::zeros(gray.size(), CV_8UC1);
                vector<Point> pts = { chessboardPoints.begin(), chessboardPoints.end() };
                fillConvexPoly(mask, &pts[0], pts.size(), Scalar(255), LINE_AA);

                // Creating an inverse mask and applying it to remove the chessboard from the original frame
                Mat inverseMask;
                bitwise_not(mask, inverseMask);
                Mat frameWithoutChessboard;
                originalFrame.copyTo(frameWithoutChessboard, inverseMask);
                // Overlaying the warped image on the original frame where the chessboard was detected
                warpedImage.copyTo(originalFrame, mask);

                // Drawing a virtual object (3D effect) if certain flags are set and exactly 54 chessboard corners are detected
                if (task_6_Flag && positionCalculatedFlag && corner_set.size() == 54) {
                    // Ensure the corner set size check is redundant here, could be removed for optimization
                    if (corner_set.size() != 54) continue;

                    // Estimate the pose of the 3D object relative to the camera
                    solvePnP(point_set, corner_set, camera_matrix, distortion_coefficient, PNP_rot_vec, PNP_tran_vec);

                    // Define 3D coordinates for a virtual house/object
                    vector<Vec3f> house = {
                        // Base
                        {-1, 1, 0}, {9, 1, 0}, {-1, -6, 0}, {9, -6, 0},
                        // Top
                        {0, 0, 7}, {9, 0, 7}, {0, -6, 7}, {9, -6, 7},
                        // Cone apex
                        {5, -3, 10}
                    };

                    // Project the 3D points to 2D image points
                    vector<Point2f> image_points;
                    projectPoints(house, PNP_rot_vec, PNP_tran_vec, camera_matrix, distortion_coefficient, image_points);

                    // Draw the base and top of the house using lines between projected points
                    // Connect the dots to draw the base square
                    for (int i = 0; i < 4; i++) { // Drawing the base and top of the house
                        line(originalFrame, image_points[i], image_points[(i + 1) % 4], Scalar(0, 0, 255), 5); // Base
                        line(originalFrame, image_points[4 + i], image_points[4 + ((i + 1) % 4)], Scalar(0, 0, 255), 5); // Top
                        line(originalFrame, image_points[i], image_points[4 + i], Scalar(0, 0, 255), 5); // Sides connecting base and top
                    }

                    // Drawing the cone on top
                    for (int i = 4; i < 8; i++) {
                        line(originalFrame, image_points[i], image_points[8], Scalar(255, 0, 255), 3); // Connect base to apex
                    }

                    // Finally, display the modified frame with the overlay and virtual object
                    imshow("Frame with Overlay and Virtual Object", originalFrame);
                }
            }
            else {
                cout << "Chessboard not detected" << endl; // Output message if chessboard is not detected
            }
        }


    }

    // Cleanup
    capdev.release(); // Release the video capture object
    destroyAllWindows(); // Close all the windows

    return 0;
}
