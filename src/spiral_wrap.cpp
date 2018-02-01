#include <spiral_wrap.h>

SpiralWrap::SpiralWrap(cv::Point2f center, float distance, float max_index) : distance(distance), center(center) {
	this->max_index = max_index;
	min_index = 0.0;
}

void SpiralWrap::evaluate( float index ) {
	position = MiscMath::From2f_xy((cv::Point2f(cos(index) * distance * index, sin(index) * distance * index)) + center, 1.0);
	velocity = cv::Point3f((-sin(index) * distance * index) + cos(index), (cos(index) * distance * index) + sin(index), 0.0);
	acceleration = cv::Point3f((-cos(index) * distance * index) - (2.0 * sin(index)), (-sin(index) * distance * index) - (2.0 * cos(index)), 0.0);
}
