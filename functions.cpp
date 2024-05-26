/*
* main.cpp
* created by Keval Visaria and Chirag Dhoka Jain
*
* This file contains all the functions that are used in main.cpp
*
*/

#include <opencv2/opencv.hpp>
#include <iostream>
#include "csv_util.h"

using namespace std;
using namespace cv;

/* This function detects chessboard corners in a provided image frame.
 * It first attempts to find corners using the findChessboardCorners function with fast checking.
 * If corners are found, it refines their positions using cornerSubPix for more accurate localization.
 * The function returns true if the chessboard pattern is found and false otherwise.
 *
 * Parameters:
 * - frame: Input image in which to find chessboard corners.
 * - corner_detected: Mat object where the detected corners will be stored (unused in this snippet).
 * - pattern_size: Size of the chessboard pattern, e.g., (8x8 squares would be Size(7,7)).
 * - corner_set: Vector to store the coordinates of the detected corners.
 *
 * Returns:
 * - bool indicating whether the chessboard pattern was found.
 */
bool getCorners(Mat frame, Mat& corner_detected, Size pattern_size, vector<Point2f>& corner_set) {

    // Clear previous corner detections
    corner_set.clear();

    // Detect chessboard corners in the frame
    bool pattern_found = findChessboardCorners(frame, pattern_size, corner_set, CALIB_CB_FAST_CHECK);

    // Convert to grayscale for sub-pixel corner detection
    Mat gray;
    cvtColor(frame, gray, COLOR_BGR2GRAY);

    // Refine corner locations if found
    if (pattern_found) {
        cornerSubPix(gray, corner_set, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 50, 0.1));
    }

    return pattern_found;
}

/* This function organizes and records the coordinates of detected chessboard corners
 * and the corresponding 3D points representing the chessboard pattern. It prints the
 * corner coordinates to the console and updates the provided lists with the new data.
 *
 * Parameters:
 * - point_set: Vector to store the 3D coordinates of the chessboard corners.
 * - point_list: A list of vectors containing the 3D points for each detected chessboard.
 * - corner_set: The detected 2D corner points in the image.
 * - corner_list: A list of vectors containing the 2D corner points for each detected chessboard.
 */
void getCoordinates(vector<Vec3f>& point_set, vector<vector<Vec3f>>& point_list,
    vector<Point2f> corner_set, vector<vector<Point2f>>& corner_list) {

    cout << endl << "Frame " << corner_list.size() + 1 << ":" << endl;

    // Clear any existing points in point_set
    point_set.clear();
    // Populate point_set with 3D coordinates of the chessboard corners
    for (int i = 0; i > -6; i--) {
        for (int j = 0; j < 9; j++) {
            point_set.push_back(Vec3f(j, i, 0));
        }
    }
    // Update the lists with the current frame's data
    point_list.push_back(point_set);
    corner_list.push_back(corner_set);

    // Print the 2D corner coordinates
    cout << "Corner coordinates:" << endl;
    for (size_t j = 0; j < corner_list.back().size(); j++) {
        cout << corner_list.back()[j] << " ";
        if ((j + 1) % 9 == 0) cout << endl; // Newline for readability
    }
    cout << "Finish Recording" << endl;
}

