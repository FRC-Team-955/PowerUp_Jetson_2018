#ifndef SPLINE_WRAP_H
#define SPLINE_WRAP_H

#include <tinysplinecpp.h>
#include <sq_derivable.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <renderer.h>
#include <waypoint.h>

class SplineWrap : public SQDerivable {
	public:
		SplineWrap(std::vector<WayPoint> points);
		SplineWrap(WayPoint a, WayPoint b, bool reverse);
		void set_ctrlpts(std::vector<cv::Point3f> points);

		void render();
		bool evaluate(float index);
	private:
		const size_t spline_dimension = 3;
		tinyspline::BSpline spline;
		tinyspline::BSpline spline_derive;
		tinyspline::BSpline spline_derive_sq;
		inline cv::Point3f eval_spline_Point3f(tinyspline::BSpline* sp, float index);
};

#endif
