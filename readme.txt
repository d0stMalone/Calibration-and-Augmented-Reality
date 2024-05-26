# CS5330 Project 4: Calibration and Augmented Reality

# Authors
- Keval Visaria
- Chirag Dhoka Jain

# Instructor
- Bruce Maxwell

------------------------------------------------------------------------------------
# Project Description

This project involves developing a program to calibrate a camera and project virtual objects onto a physical chessboard using augmented reality techniques. The program utilizes calibration images of a chessboard to compute the camera matrix and distortion coefficients. It then uses these parameters to accurately overlay 3D objects and images onto the chessboard in real-time.

-------------------------------------------------------------------------------------
# File Structure

- `main.cpp`: Main program that calibrates the camera and projects objects onto a chessboard.
- `functions.cpp` / `calibrationFunctions.h`: Functions used in the main program.
- `SURF.cpp`: Displays corners detected in the calibration images.
- `data.csv`: Stores the camera matrix and distortion coefficients obtained from the calibration process.

---------------------------------------------------------------------------------------
# Setup Instructions

1. Ensure you have the required libraries installed for image processing and augmented reality (e.g., OpenCV).
2. Compile the source code files (`main.cpp`, `functions.cpp`, `SURF.cpp`) using your preferred C++ compiler and linking against the necessary libraries.
3. Place a chessboard in view of the camera for calibration and augmented reality projection.

--------------------------------------------------------------------------------------
# Usage

## Calibration and Augmented Reality (main.cpp) 

1. Point the camera towards the chessboard.
2. Press `'s'` to save the current frame as a calibration image.
3. After storing at least 5 calibration images, press `'c'` to calibrate the camera and display the RMS reprojection error.
4. Press `'m'` to compute the rotation and translation matrices.
5. Press `'a'` to overlay 3D axes on the chessboard.
6. Press `'v'` to display a virtual object on the chessboard.
7. Press `'d'` to show an image on the chessboard.
8. Press `'t'` to display both the image and virtual object on the chessboard.
9. Press `'q'` to quit the program.

## Feature Detection (SURF.cpp)

1. Point the camera towards the chessboard or any other surface/object.
2. A window showing all detected features will be displayed.
3. Press `'q'` to quit the program.

---------------------------------------------------------------------------------------



