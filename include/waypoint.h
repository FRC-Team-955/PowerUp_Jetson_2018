#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <opencv2/opencv.hpp>
#include <misc_math.h>

struct WayPoint {
	cv::Point2f position;
	float velocity_beginning,
			velocity_end,
			direction,
			length;
	void to_control_points (std::vector<cv::Point3f> &output);
};

#endif
