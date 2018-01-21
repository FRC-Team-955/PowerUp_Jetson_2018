#include <sine_wrap.h>

/*
SineWrap::SineWrap(cv::Point2f beginning, float amplitude, float period, size_t repeats) {
	this->amplitude = amplitude;
	this->period = period;
	this->beginning = beginning;
	this->repeats = repeats;
	advance(0.0);
}

bool SineWrap::advance(float index) {
	if (function_index + index <= (float)repeats * MiscMath::pi && function_index + index >= 0.0) {
		function_index += index;
		position = cv::Point3f(function_index * period, sin(function_index) * amplitude, 1.0) + MiscMath::From2f_xy(beginning, 0.0);
		velocity = cv::Point3f(period, cos(function_index) * amplitude, 1.0);
		acceleration = cv::Point3f(0.0, -sin(function_index) * amplitude, 0.0);
		return true;
	} else {
		return false;
	}
}

bool SineWrap::is_at_beginning() {
	return function_index == 0.0;
}
*/
