// https://learnopencv.com/camera-calibration-using-opencv/
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#include <stdio.h>
#include <iostream>

#include "../include/CameraCalibrator.hpp"

// Defining the dimensions of checkerboard
int CHECKERBOARD[2]{6,9};

CameraCalibrator::CameraCalibrator(int noFrames) {
    CameraCalibrator::calibrationFrames = noFrames;

    // Check for calib file
}

void CameraCalibrator::calibrate(cv::VideoCapture &videoCapture) {
    for(int i{0}; i<CHECKERBOARD[1]; i++)
    {
        for(int j{0}; j<CHECKERBOARD[0]; j++)
            CameraCalibrator::objp.push_back(cv::Point3f(j,i,0));
    }

    cv::Mat frame, gray;
    // vector to store the pixel coordinates of detected checkerboard corners
    std::vector<cv::Point2f> corner_pts;
    bool success;

    // Looping over all the images in the directory
    for(int i{0}; i<CameraCalibrator::calibrationFrames; i++)
    {
        do {
            frame = CameraCalibrator::nextImage(videoCapture);
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

            // Finding checkerboard corners
            // If desired number of corners are found in the image then success = true
            success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts,
                                                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK |
                                                cv::CALIB_CB_NORMALIZE_IMAGE);
        } while(!success);
        /*
         * Refine the pixel coordinates and display
         * them on the images of checkerboard
        */

        cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

        // refining pixel coordinates for given 2d points.
        cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

        // Displaying the detected corner points on the checkerboard
        cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

        CameraCalibrator::objpoints.push_back(objp);
        CameraCalibrator::imgpoints.push_back(corner_pts);

        cv::imshow("Image", frame);
        cv::waitKey(0);
    }

    cv::destroyAllWindows();

    cv::Mat cameraMatrix,distCoeffs,R,T;

    /*
     * Performing camera calibration by
     * passing the value of known 3D points (objpoints)
     * and corresponding pixel coordinates of the
     * detected corners (imgpoints)
    */
    cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows,gray.cols), cameraMatrix, distCoeffs, R, T);

    std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
    std::cout << "distCoeffs : " << distCoeffs << std::endl;
    std::cout << "Rotation vector : " << R << std::endl;
    std::cout << "Translation vector : " << T << std::endl;

    // TODO: Save to json file
}

cv::Mat CameraCalibrator::nextImage(cv::VideoCapture &videoCapture) {
    cv::Mat result;
    if (videoCapture.isOpened()) {
        cv::Mat view0;
        videoCapture >> view0;
        view0.copyTo(result);
    }
    else {
        // TODO: handle error case?
//        throw
    }

    return result;
}
