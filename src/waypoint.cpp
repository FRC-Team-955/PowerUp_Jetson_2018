#include <waypoint.h>

void WayPoint::to_control_points (std::vector<cv::Point3f> &output, bool invert) {
	output.push_back(MM::From2f_xy(this->position, this->velocity_beginning));
	output.push_back(MM::From2f_xy(
				MM::RadialOffset(((reverse != invert) ? MM::pi : 0.0) + this->direction, this->length / 2.0, this->position), 
				this->velocity_end));
	output.push_back(MM::From2f_xy(
				MM::RadialOffset(((reverse != invert) ? MM::pi : 0.0) + this->direction, this->length, this->position), 
				this->velocity_end));
	if (invert) {
		std::reverse(output.end() - 3, output.end());
	}
}
