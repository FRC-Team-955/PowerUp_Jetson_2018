#include <spline_wrap.h>

SplineWrap::SplineWrap(WayPoint a, WayPoint b) {
	bool reverse = 
		a.velocity_end < 0 ||
		a.velocity_beginning < 0 ||
		b.velocity_end < 0 ||
		b.velocity_beginning < 0;

	stop_index = 1.0;
	start_index = 0.0;
	if (reverse) {
		std::swap(a, b);
		std::swap(start_index, stop_index);
	}

	std::vector<cv::Point3f> control_points;
	a.to_control_points(control_points, false);
	b.to_control_points(control_points, true);
	set_ctrlpts(control_points);
}

SplineWrap::SplineWrap(std::vector<WayPoint> waypoints) {
	std::vector<cv::Point3f> control_points;
	for (auto& waypoint : waypoints)
		waypoint.to_control_points(control_points, false);
	set_ctrlpts(control_points);
}

void SplineWrap::set_ctrlpts(std::vector<cv::Point3f> control_points) {
	spline = tinyspline::BSpline(control_points.size(), spline_dimension, 3);
	std::vector<tinyspline::real> ctrlp;
	for (auto& point : control_points) {
		ctrlp.push_back(point.x);
		ctrlp.push_back(point.y);
		ctrlp.push_back(point.z);
	}
	spline.setCtrlp(ctrlp);
	spline_derive = spline.derive();
	spline_derive_sq = spline_derive.derive();
	evaluate(0.0);
}

bool SplineWrap::evaluate(float index) {
	if (within_bounds(index)) {
		position = eval_spline_Point3f(&spline, index);
		velocity = eval_spline_Point3f(&spline_derive, index);
		acceleration = eval_spline_Point3f(&spline_derive_sq, index);
		return true;
	} else {
		return false;
	}
}

inline cv::Point3f SplineWrap::eval_spline_Point3f(tinyspline::BSpline* sp, float change_in_index) {
	auto eval = sp->evaluate(change_in_index).result();
	return cv::Point3f(eval[0], eval[1], eval[2]);
}

void SplineWrap::render() {
	Renderer::render_spline(&spline);
}
