#include <waypoint.h>

void WayPoint::to_control_points (std::vector<cv::Point3f> &output) {
	output.push_back(MiscMath::From2f_xy(this->position, this->velocity_beginning));
	output.push_back(MiscMath::From2f_xy(
				MiscMath::RadialOffset(this->direction, this->length / 2.0, this->position), 
				this->velocity_end));
	output.push_back(MiscMath::From2f_xy(
				MiscMath::RadialOffset(this->direction, this->length, this->position), 
				this->velocity_end));
}
