#include <spline_wrap.h>

SplineWrap::SplineWrap(int nCtrlp) {
	max_index = 1.0;
	min_index = 0.0;
	spline = tinyspline::BSpline(nCtrlp, spline_dimension, 3);
}

bool SplineWrap::set_ctrlpts(std::vector<cv::Point3f> points) {
	if (points.size() != spline.nCtrlp())
		return false;
	std::vector<tinyspline::real> ctrlp;
	for (auto& point : points) {
		ctrlp.push_back(point.x);
		ctrlp.push_back(point.y);
		ctrlp.push_back(point.z);
	}
	spline.setCtrlp(ctrlp);
	spline_derive = spline.derive();
	spline_derive_sq = spline_derive.derive();
	index = min_index;
	advance(min_index);
	return true;
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
