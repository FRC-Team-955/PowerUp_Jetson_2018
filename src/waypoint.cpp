#include <waypoint.h>

//Create control points for a spline
void WayPoint::to_control_points (std::vector<cv::Point3f> &output, bool invert) {
	output.push_back(MM::From2f_xy(this->position, this->velocity_beginning));
	output.push_back(MM::From2f_xy(
				MM::RadialOffset((invert ? MM::pi : 0.0) + this->direction, this->length / 2.0, this->position), 
				this->velocity_end));
	output.push_back(MM::From2f_xy(
				MM::RadialOffset((invert ? MM::pi : 0.0) + this->direction, this->length, this->position), 
				this->velocity_end));
	if (invert) {
		std::reverse(output.end() - 3, output.end());
	}
}

//Return a waypoint behind this one, but moved along it's direction by _wheel width_ units
WayPoint WayPoint::before(float distance) {
	WayPoint copy = *this;
	copy.position = MiscMath::RadialOffset(this->direction, -distance / 2.0, this->position);
	return copy;
}

void WayPoint::render(bool invert) {
	glColor3f(1.0, 0.7, 0.0);
	glLineWidth(3);
	std::vector<cv::Point2f> wireframe;
	wireframe.push_back(cv::Point2f (0.0, this->length * 0.5));
	wireframe.push_back(cv::Point2f (0.0, 0.0));

	wireframe.push_back(cv::Point2f (0.0, this->length * 0.5));
	wireframe.push_back(cv::Point2f (this->length * 0.1, this->length * 0.4));

	wireframe.push_back(cv::Point2f (0.0, this->length * 0.5));
	wireframe.push_back(cv::Point2f (this->length * -0.1, this->length * 0.4));

	cv::Mat rot_mat( 2, 3, CV_32FC1 );
	rot_mat = cv::getRotationMatrix2D(cv::Point2f(0.0, 0.0), (-this->direction * (180.0 / acos(-1))) + 90.0, 1.0);
	cv::transform(wireframe, wireframe, rot_mat);

	cv::Point2f last = wireframe.back();
	glBegin(GL_LINES);
	for (auto& point : wireframe) {
		glVertex2f(point.x + position.x, point.y + position.y);
		glVertex2f(last.x + position.x, last.y + position.y);
		last = point;
	}



}
