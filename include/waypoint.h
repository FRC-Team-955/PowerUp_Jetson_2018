#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <opencv2/opencv.hpp>
#include <misc_math.h>

namespace MM = MiscMath;

struct WayPoint {
	cv::Point2f position;
	float velocity_beginning,
			velocity_end,
			direction,
			length;
	bool reverse; //Flip the direction of this point
	void to_control_points (std::vector<cv::Point3f> &output, bool invert);
};

#endif
