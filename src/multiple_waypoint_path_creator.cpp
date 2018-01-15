#include <multiple_waypoint_path_creator.h>

MultipleWaypointPathCreator::MultipleWaypointPathCreator(int nCtrlp, float max_time_step, float wheel_distance) {
	size_t max_ctrlp = nCtrlp;
	spline = tinyspline::BSpline(nCtrlp * 3, 3, 3);
	ctrlp = spline.ctrlp();
	this->max_time_step = max_time_step; 
	this->wheel_distance = wheel_distance;
}

void MultipleWaypointPathCreator::clear_points() {
	spline_iter = 0;
	//ctrlp.clear();
}

bool MultipleWaypointPathCreator::push_point_direction(cv::Point2f position, float direction, float speed_start, float speed_end) {
	//if (spline_iter < max_ctrlp * spline.dim()) {
		ctrlp[spline_iter++] = position.x;
		ctrlp[spline_iter++] = position.y;
		ctrlp[spline_iter++] = speed_start;

		auto leading_a_start = MiscMath::RadialOffset(direction, wheel_distance / 2.0, position);
		ctrlp[spline_iter++] = leading_a_start.x;
		ctrlp[spline_iter++] = leading_a_start.y;
		ctrlp[spline_iter++] = speed_end; //TODO: Make this configurable?

		auto leading_b_start = MiscMath::RadialOffset(direction, wheel_distance, position);
		ctrlp[spline_iter++] = leading_b_start.x;
		ctrlp[spline_iter++] = leading_b_start.y;
		ctrlp[spline_iter++] = speed_end;
		return true; //TODO: Check the spline index for sanity
}

Path MultipleWaypointPathCreator::calculate_path() {
	spline.setCtrlp(ctrlp);
	return Path(spline, wheel_distance, max_time_step);
}

