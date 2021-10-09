#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>


class CameraCalibrator
{
public:
    CameraCalibrator(int noFrames);
    ~CameraCalibrator();
    void calibrate(cv::VideoCapture &videoCapture);

private:
    int calibrationFrames = 15;

    cv::VideoCapture inputCapture;
    cv::Mat nextImage(cv::VideoCapture &videoCapture);
    // Creating vector to store vectors of 3D points for each checkerboard image
    std::vector<std::vector<cv::Point3f> > objpoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f> > imgpoints;

    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
};
