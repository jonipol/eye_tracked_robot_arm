#pragma once
#include <tobii/tobii.h>
#include <tobii/tobii_streams.h>
#include <opencv2/core.hpp>

class EyeTracking
{
public:
	EyeTracking();
	cv::Point2f getGazePoint();
	tobii_gaze_point_t getGazePoint_t();
	void cleanup();

private:
	tobii_api_t* api = NULL;
	char url[256] = { 0 };
	tobii_device_t* device = NULL;
	tobii_error_t result;
	tobii_gaze_point_t latestGazePoint;

    void url_receiver(char const* url, void* user_data)
    void gaze_point_callback(tobii_gaze_point_t const* gaze_point, void* user_data)
	void getAndProcessGazePoint();
};