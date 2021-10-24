#include "gaze_controller/gaze_to_world_coordinates.hpp"

#include <iostream>
#include <opencv2/opencv.hpp>
using std::placeholders::_1;


//@ref: http://answers.opencv.org/question/67008/can-i-get-2d-world-coordinates-from-a-single-image-uv-coords/


double checkCameraPose(const std::vector<cv::Point3f> &modelPts, const std::vector<cv::Point2f> &imagePts, const cv::Mat &cameraMatrix,
                       const cv::Mat &distCoeffs, const cv::Mat &rvec, const cv::Mat &tvec) {
    std::vector<cv::Point2f> projectedPts;
    cv::projectPoints(modelPts, rvec, tvec, cameraMatrix, distCoeffs, projectedPts);

    double rms = 0.0;
    for (size_t i = 0; i < projectedPts.size(); i++) {
        rms += (projectedPts[i].x-imagePts[i].x)*(projectedPts[i].x-imagePts[i].x) + (projectedPts[i].y-imagePts[i].y)*(projectedPts[i].y-imagePts[i].y);
    }

    return sqrt(rms / projectedPts.size());
}

cv::Point3f transformPoint(const cv::Point3f &pt, const cv::Mat &rvec, const cv::Mat &tvec) {
    //Compute res = (R | T) . pt
    cv::Mat rotationMatrix;
    cv::Rodrigues(rvec, rotationMatrix);

    cv::Mat transformationMatrix = (cv::Mat_<double>(4, 4) << rotationMatrix.at<double>(0,0), rotationMatrix.at<double>(0,1), rotationMatrix.at<double>(0,2), tvec.at<double>(0),
            rotationMatrix.at<double>(1,0), rotationMatrix.at<double>(1,1), rotationMatrix.at<double>(1,2), tvec.at<double>(1),
            rotationMatrix.at<double>(2,0), rotationMatrix.at<double>(2,1), rotationMatrix.at<double>(2,2), tvec.at<double>(2),
            0, 0, 0, 1);

    cv::Mat homogeneousPt = (cv::Mat_<double>(4, 1) << pt.x, pt.y, pt.z, 1.0);
    cv::Mat transformedPtMat = transformationMatrix * homogeneousPt;

    cv::Point3f transformedPt(transformedPtMat.at<double>(0), transformedPtMat.at<double>(1), transformedPtMat.at<double>(2));
    return transformedPt;
}

cv::Point3f transformPointInverse(const cv::Point3f &pt, const cv::Mat &rvec, const cv::Mat &tvec) {
    //Compute res = (R^t | -R^t . T) . pt
    cv::Mat rotationMatrix;
    cv::Rodrigues(rvec, rotationMatrix);
    rotationMatrix = rotationMatrix.t();

    cv::Mat translation = -rotationMatrix*tvec;

    cv::Mat transformationMatrix = (cv::Mat_<double>(4, 4) << rotationMatrix.at<double>(0,0), rotationMatrix.at<double>(0,1), rotationMatrix.at<double>(0,2), translation.at<double>(0),
            rotationMatrix.at<double>(1,0), rotationMatrix.at<double>(1,1), rotationMatrix.at<double>(1,2), translation.at<double>(1),
            rotationMatrix.at<double>(2,0), rotationMatrix.at<double>(2,1), rotationMatrix.at<double>(2,2), translation.at<double>(2),
            0, 0, 0, 1);

    cv::Mat homogeneousPt = (cv::Mat_<double>(4, 1) << pt.x, pt.y, pt.z, 1.0);
    cv::Mat transformedPtMat = transformationMatrix * homogeneousPt;

    cv::Point3f transformedPt(transformedPtMat.at<double>(0), transformedPtMat.at<double>(1), transformedPtMat.at<double>(2));
    return transformedPt;
}

void computePlaneEquation(const cv::Point3f &p0, const cv::Point3f &p1, const cv::Point3f &p2, float &a, float &b, float &c, float &d) {
    //Vector p0_p1
    cv::Point3f p0_p1;
    p0_p1.x = p0.x - p1.x;
    p0_p1.y = p0.y - p1.y;
    p0_p1.z = p0.z - p1.z;

    //Vector p0_p2
    cv::Point3f p0_p2;
    p0_p2.x = p0.x - p2.x;
    p0_p2.y = p0.y - p2.y;
    p0_p2.z = p0.z - p2.z;

    //Normal vector
    cv::Point3f n = p0_p1.cross(p0_p2);

    a = n.x;
    b = n.y;
    c = n.z;
    d = -(a*p0.x + b*p0.y + c*p0.z);

    float norm =  sqrt(a*a + b*b + c*c);
    a /= norm;
    b /= norm;
    c /= norm;
    d /= norm;
}

cv::Point3f compute3DOnPlaneFrom2D(const cv::Point2f &imagePt, const cv::Mat &cameraMatrix, const float a, const float b, const float c, const float d) {
    double fx = cameraMatrix.at<double>(0,0);
    double fy = cameraMatrix.at<double>(1,1);
    double cx = cameraMatrix.at<double>(0,2);
    double cy = cameraMatrix.at<double>(1,2);

    cv::Point2f normalizedImagePt;
    normalizedImagePt.x = (imagePt.x - cx) / fx;
    normalizedImagePt.y = (imagePt.y - cy) / fy;

    float s = -d / (a*normalizedImagePt.x + b*normalizedImagePt.y + c);

    cv::Point3f pt;
    pt.x = s*normalizedImagePt.x;
    pt.y = s*normalizedImagePt.y;
    pt.z = s;

    return pt;
}

void extract_camera_intrinsics(sensor_msgs::msg::CameraInfo camera_info, cv::Mat &camera_matrix, cv::Mat &dist_coeffs) {
    camera_matrix = cv::Mat_<float>(3, 3);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; i++) {
            camera_matrix.at<float>(i, j) = camera_info.k[i + j];
        }
    }

    dist_coeffs = cv::Mat_<float>(5, 1);
    for (int i = 0; i < 5; i++) {
        dist_coeffs.at<float>(i) = camera_info.d[i];
    }
}

geometry_msgs::msg::Point gaze_to_world_coordinates::calculate_world_point_from_gaze(apriltag_msgs::msg::AprilTagDetectionArray &tag_array) {
    // Get tag_centers
    std::vector<cv::Point2f> pointbuf;

    // Match the ids with predefined tags

    // Need 3 to get plane
    if (pointbuf.size() < 3)
    // TODO: Define the exception
        throw;

    // Camera pose
    cv::Mat rvec, tvec;
    cv::solvePnP(modelPts_, pointbuf, cameraMatrix_, distCoeffs_, rvec, tvec);

    //Transform model point (in object frame) to the camera frame
    cv::Point3f pt0 = transformPoint(modelPts_[0], rvec, tvec);
    cv::Point3f pt1 = transformPoint(modelPts_[1], rvec, tvec);
    cv::Point3f pt2 = transformPoint(modelPts_[2], rvec, tvec);

    //Compute plane equation in the camera frame
    // TODO: Plane class for clarity maybe?
    float a, b, c, d;
    computePlaneEquation(pt0, pt1, pt2, a, b, c, d);
    std::cout << "Plane equation=" << a << " ; " << b << " ; " << c << " ; " << d << std::endl;

    //Compute 3D from 2D DEBUG
    std::vector<cv::Point3f> pts3dCameraFrame, pts3dObjectFrame;
    double rms_3D = 0.0;
    for (size_t i = 0; i < pointbuf.size(); i++) {
        cv::Point3f pt = compute3DOnPlaneFrom2D(pointbuf[i], cameraMatrix_, a, b, c, d);
        pts3dCameraFrame.push_back(pt);

        cv::Point3f ptObjectFrame = transformPointInverse(pt, rvec, tvec);
        pts3dObjectFrame.push_back(ptObjectFrame);

        rms_3D += (modelPts_[i].x-ptObjectFrame.x)*(modelPts_[i].x-ptObjectFrame.x) + (modelPts_[i].y-ptObjectFrame.y)*(modelPts_[i].y-ptObjectFrame.y) +
                  (modelPts_[i].z-ptObjectFrame.z)*(modelPts_[i].z-ptObjectFrame.z);

        std::cout << "modelPts[" << i << "]=" << modelPts_[i] << " ; calc=" << ptObjectFrame << std::endl;
    }

    std::cout << "RMS error for model points=" << sqrt(rms_3D / pointbuf.size()) << std::endl;

    // Compute 3D from 2D point
    cv::Point2f gaze_pixel = ros_point_to_pixel(latest_gaze_);
    cv::Point3f point_in_camera_frame = compute3DOnPlaneFrom2D(gaze_pixel, cameraMatrix_, a, b, c, d);
    cv::Point3f point_in_object_frame = transformPointInverse(point_in_camera_frame, rvec, tvec);

    return point3f_to_ros_point(point_in_object_frame);
}

cv::Point2f gaze_to_world_coordinates::ros_point_to_pixel(geometry_msgs::msg::PointStamped ros_point) {
    cv::Point2f point;
    point.x = ros_point.point.x;
    point.y = ros_point.point.y;
    return point;
}

geometry_msgs::msg::Point gaze_to_world_coordinates::point3f_to_ros_point(cv::Point3f cv_point) {
    geometry_msgs::msg::Point point;
    point.x = cv_point.x;
    point.y = cv_point.y;
    point.z = cv_point.z;
    return point;
}

gaze_to_world_coordinates::gaze_to_world_coordinates(const char * node_name) : Node(node_name) {
    this->declare_parameter("gaze_topic", "/gaze");
    this->declare_parameter("camera_info_topic", "/camera_info");
    this->declare_parameter("tag_topic", "/tag_detections");
    std::string gaze_topic = this->get_parameter("gaze_topic").as_string();
    std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();
    std::string tag_topic = this->get_parameter("tag_topic").as_string();

    gaze_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            gaze_topic, sensor_qos_, std::bind(&gaze_to_world_coordinates::gaze_callback, this, _1));

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic, sensor_qos_, std::bind(&gaze_to_world_coordinates::camera_info_callback, this, _1));

    tag_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            tag_topic, sensor_qos_, std::bind(&gaze_to_world_coordinates::tag_callback, this, _1));

    world_point_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/world_point", sensor_qos_);

    // TODO: Get points as parameters.
    // TODO: Make points reconfigurable
    // TODO: Make amount dynamic
    //          TODO: Make sure there is at least 3 points
    modelPts_.push_back(cv::Point3f(5.0, 5.0, 0.0));
    modelPts_.push_back(cv::Point3f(5.0, -5.0, 0.0));
    modelPts_.push_back(cv::Point3f(-5.0, -5.0, 0.0));
    modelPts_.push_back(cv::Point3f(-5.0, 5.0, 0.0));
}

void gaze_to_world_coordinates::gaze_callback(geometry_msgs::msg::PointStamped::SharedPtr gaze_point) {
    latest_gaze_ = *gaze_point;
}

void gaze_to_world_coordinates::camera_info_callback(sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {
    extract_camera_intrinsics(*camera_info, cameraMatrix_, distCoeffs_);
}

void gaze_to_world_coordinates::tag_callback(apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr tag_array) {
    // Check timestamp
    if (tag_array->header.stamp.sec - latest_gaze_.header.stamp.sec > time_tolerance_) {
        return;
    }

    if (tag_array->detections.size() < 3) {
        return;
    }

    // Calculate world point
    try {
        auto point = calculate_world_point_from_gaze(*tag_array);

        // Publish world point
        world_point_pub_->publish(point);
    }
    catch (...) {
//    catch (const std::exception& e) {
        RCLCPP_DEBUG(this->get_logger(), "Not enough detected tags");
    }

}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    // TODO: Multi-threaded executor
    auto gaze_to_world_node = std::make_shared<gaze_to_world_coordinates>("gaze_to_world_node");
    rclcpp::spin(gaze_to_world_node);
    rclcpp::shutdown();
}
