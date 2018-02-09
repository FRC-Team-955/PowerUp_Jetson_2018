#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <opencv2/opencv.hpp>
#include <misc_math.h>

#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>

namespace MM = MiscMath;

struct WayPoint {
	cv::Point2f position;
	float velocity_beginning,
			velocity_end,
			direction,
			length;
	WayPoint before ();
	void to_control_points (std::vector<cv::Point3f> &output, bool invert);
	void render(bool invert);
};

#endif
