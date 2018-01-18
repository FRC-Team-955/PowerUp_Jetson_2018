#include <spline_wrap.h>

SplineWrap::SplineWrap(int nCtrlp, std::vector<cv::Point3f> points) {
	spline = tinyspline::BSpline(nCtrlp, spline_dimension, 3);
	if (!set_ctrlpts(points)) //TODO: Real error handling
		ErrorPrinting::print_error("failed to set control points (Not enough!)");
}

bool SplineWrap::set_ctrlpts(std::vector<cv::Point3f> points) {
	if (points.size() != spline.nCtrlp()) {
		std::cout << points.size() << " " << spline_dimension << " " << spline.nCtrlp() << std::endl;
		return false;
	}
	std::vector<tinyspline::real> ctrlp;
	for (auto& point : points) {
		ctrlp.push_back(point.x);
		ctrlp.push_back(point.y);
		ctrlp.push_back(point.z);
	}
	spline.setCtrlp(ctrlp);
	spline_derive = spline.derive();
	spline_derive_sq = spline_derive.derive();
	function_index = 0.0;
	advance(0.0);
	return true;
}

bool SplineWrap::advance(float index) {
	if (function_index + index <= 1.0 && function_index + index >= 0.0) {
		function_index += index;
		position = eval_spline_Point3f(&spline, function_index);
		velocity = eval_spline_Point3f(&spline_derive, function_index);
		acceleration = eval_spline_Point3f(&spline_derive_sq, function_index);
		return true;
	} else {
		return false;
	}
}

bool SplineWrap::is_at_end() {
	return fabs(function_index - 1.0) <= 0.01;
}

bool SplineWrap::is_at_beginning() {
	return function_index == 0.0;
}

cv::Point3f SplineWrap::eval_spline_Point3f(tinyspline::BSpline* sp, float index) {
	auto eval = sp->evaluate(index).result();
	return cv::Point3f(eval[0], eval[1], eval[2]);
}

void SplineWrap::render() {
	Renderer::render_spline(&spline);
}
