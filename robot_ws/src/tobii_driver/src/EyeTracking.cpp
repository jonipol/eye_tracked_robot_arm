#include <iostream>
#include <assert.h>
#include "../include/EyeTracking.hpp"

void EyeTracking::gaze_point_callback(tobii_gaze_point_t const* gaze_point, void* user_data)
{
	if (gaze_point->validity == TOBII_VALIDITY_VALID) {
		tobii_gaze_point_t* gaze_point_storage = (tobii_gaze_point_t*)user_data;
		*gaze_point_storage = *gaze_point;
	}
}

void EyeTracking::url_receiver(char const* url, void* user_data)
{
	char* buffer = (char*)user_data;
	if (*buffer != '\0') return;

	if (strlen(url) < 256)
		strcpy_s(buffer, 256, url);
}

EyeTracking::EyeTracking()
{
	// Eye_traking API init
	result = tobii_api_create(&api, NULL, NULL);
	assert(result == TOBII_ERROR_NO_ERROR);

	result = tobii_enumerate_local_device_urls(api, url_receiver, url);
	assert(result == TOBII_ERROR_NO_ERROR);
	if (*url == '\0')
	{
		printf("Error: No device found\n");
		throw;
	}

	// Connect to first tracker found
	result = tobii_device_create(api, url, TOBII_FIELD_OF_USE_INTERACTIVE, &device);
	assert(result == TOBII_ERROR_NO_ERROR);
	// Subscribe to gaze data
	result = tobii_gaze_point_subscribe(device, gaze_point_callback, &latestGazePoint);
	assert(result == TOBII_ERROR_NO_ERROR);

	std::cout << "End of EyeTracking init" << std::endl;
}


void EyeTracking::getAndProcessGazePoint()
{
	result = tobii_wait_for_callbacks(1, &device);
	assert(result == TOBII_ERROR_NO_ERROR || result == TOBII_ERROR_TIMED_OUT);
	result = tobii_device_process_callbacks(device);
	assert(result == TOBII_ERROR_NO_ERROR);
}

cv::Point2f EyeTracking::getGazePoint()
{
	getAndProcessGazePoint();
	// latestGazePoint.timestamp_us available if needed
	return cv::Point2f(latestGazePoint.position_xy[0], latestGazePoint.position_xy[1]);
}

tobii_gaze_point_t EyeTracking::getGazePoint_t()
{
	getAndProcessGazePoint();
	return latestGazePoint;
}

void EyeTracking::cleanup()
{
	tobii_error_t result = tobii_gaze_point_unsubscribe(device);
	assert(result == TOBII_ERROR_NO_ERROR);
	result = tobii_device_destroy(device);
	assert(result == TOBII_ERROR_NO_ERROR);
	result = tobii_api_destroy(api);
	assert(result == TOBII_ERROR_NO_ERROR);
}