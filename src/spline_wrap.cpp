#include <spline_wrap.h>

SplineWrap::SplineWrap(size_t nCtrlp) {
}

SplineWrap::SplineWrap(SplineWrap::WayPoint a, SplineWrap::WayPoint b) {
	max_index = 1.0;
	min_index = 0.0;
	std::vector<cv::Point3f> control_points;
	a.to_control_points(control_points);
	b.to_control_points(control_points);
	SplineWrap(control_points.size());	
	set_ctrlpts(control_points);
}

SplineWrap::SplineWrap(std::vector<SplineWrap::WayPoint> waypoints) {
	std::vector<cv::Point3f> control_points;
	for (auto& waypoint : waypoints)
		waypoint.to_control_points(control_points);
	SplineWrap(control_points.size());	
	set_ctrlpts(control_points);
}

void SplineWrap::WayPoint::to_control_points (std::vector<cv::Point3f> &output) {
	output.push_back(MiscMath::From2f_xy(this->position, this->velocity_beginning));
	output.push_back(MiscMath::From2f_xy(
				MiscMath::RadialOffset(this->direction, this->length / 2.0, this->position), 
				this->velocity_end));
	output.push_back(MiscMath::From2f_xy(
				MiscMath::RadialOffset(this->direction, this->length, this->position), 
				this->velocity_end));
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
	index = min_index;
	advance(min_index);
}

bool SplineWrap::advance(float change_in_index) {
	if (index + change_in_index <= max_index && index + change_in_index >= min_index) {
		index += change_in_index;
		position = eval_spline_Point3f(&spline, index);
		velocity = eval_spline_Point3f(&spline_derive, index);
		acceleration = eval_spline_Point3f(&spline_derive_sq, index);
		return true;
	} else {
		return false;
	}
}

cv::Point3f SplineWrap::eval_spline_Point3f(tinyspline::BSpline* sp, float change_in_index) {
	auto eval = sp->evaluate(change_in_index).result();
	return cv::Point3f(eval[0], eval[1], eval[2]);
}

void SplineWrap::render() {
	Renderer::render_spline(&spline);
}
