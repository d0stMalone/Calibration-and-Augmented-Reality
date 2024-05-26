


#ifndef CORNER_DETECTION_H
#define CORNER_DETECTION_H

#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;

/*
 * Function: getCorners
 * --------------------
 * Detects chessboard corners in an image frame. If corners are found,
 * it refines their positions for increased accuracy.
 *
 * Parameters:
 * - frame: The input image in which to find chessboard corners.
 * - corner_detected: Output parameter that is intended to store the detected corners
 * - pattern_size: The dimensions of the chessboard pattern (e.g., for an 8x8 chessboard, use Size(7,7)).
 * - corner_set: Vector to be populated with the precise coordinates of the detected corners.
 *
 * Returns:
 * - bool: True if the chessboard pattern was found and corners were detected, false otherwise.
 */
bool getCorners(Mat frame, Mat& corner_detected, Size pattern_size, vector<Point2f>& corner_set);

/*
 * Function: getCoordinates
 * -------------------------
 * Organizes the 3D world points corresponding to the chessboard pattern and the 2D image points
 * of the detected corners. It facilitates camera calibration and augmented reality applications
 * by mapping between 3D and 2D spaces.
 *
 * Parameters:
 * - point_set: Vector to store the 3D coordinates representing the corners of the chessboard.
 * - point_list: A list of vectors, each containing the 3D points for a detected chessboard in different frames.
 * - corner_set: The detected 2D corner points in the image.
 * - corner_list: A list of vectors, each containing the 2D corner points for a detected chessboard in different frames.
 *
 * This function does not return a value but updates the provided lists with the new data.
 */
void getCoordinates(vector<Vec3f>& point_set, vector<vector<Vec3f>>& point_list, vector<Point2f> corner_set, vector<vector<Point2f>>& corner_list);

#endif // CORNER_DETECTION_H
