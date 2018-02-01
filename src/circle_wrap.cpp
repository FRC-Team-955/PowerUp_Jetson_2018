#include <circle_wrap.h>

CircleWrap::CircleWrap(cv::Point2f center, float y_distance, float x_distance) : y_distance(y_distance), x_distance(x_distance), center(center) {
	stop_index = MiscMath::pi * 2.0;
	start_index = 0.0;
}

bool CircleWrap::evaluate( float index ) {
	if (within_bounds(index)) {
		position = MiscMath::From2f_xy((cv::Point2f(cos(index) * x_distance, sin(index) * y_distance)) + center, 1.0);
		velocity = cv::Point3f(-sin(index) * x_distance, cos(index) * y_distance, 0.0);
		acceleration = cv::Point3f(-cos(index) * x_distance, -sin(index) * y_distance, 0.0);
		return true;
	} else {
		return false;
	}
}
