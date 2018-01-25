#include <spline_wrap.h>

SplineWrap::SplineWrap(WayPoint a, WayPoint b) {
	max_index = 1.0;
	min_index = 0.0;
	std::vector<cv::Point3f> control_points;
	a.to_control_points(control_points);
	b.to_control_points(control_points);
	set_ctrlpts(control_points);
}

SplineWrap::SplineWrap(std::vector<WayPoint> waypoints) {
	std::vector<cv::Point3f> control_points;
	for (auto& waypoint : waypoints)
		waypoint.to_control_points(control_points);
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

void SplineWrap::evaluate(float index) {
	position = eval_spline_Point3f(&spline, index);
	velocity = eval_spline_Point3f(&spline_derive, index);
	acceleration = eval_spline_Point3f(&spline_derive_sq, index);
}

cv::Point3f SplineWrap::eval_spline_Point3f(tinyspline::BSpline* sp, float change_in_index) {
	auto eval = sp->evaluate(change_in_index).result();
	return cv::Point3f(eval[0], eval[1], eval[2]);
}

void SplineWrap::render() {
	Renderer::render_spline(&spline);
}
