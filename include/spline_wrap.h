#ifndef SPLINE_WRAP_H
#define SPLINE_WRAP_H

#include <tinysplinecpp.h>
#include <sq_derivable.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <renderer.h>

class SplineWrap : public SQDerivable {
	public:
		SplineWrap(size_t nCtrlp);
		bool advance(float change_in_index);
		struct WayPoint {
			cv::Point2f position;
			float velocity_beginning,
					velocity_end,
					direction,
					length;
			void to_control_points (std::vector<cv::Point3f> &output);
		};

		SplineWrap(std::vector<WayPoint> points);
		SplineWrap(SplineWrap::WayPoint a, SplineWrap::WayPoint b);
		void set_ctrlpts(std::vector<cv::Point3f> points);

		void render();
	private:
		const size_t spline_dimension = 3;
		tinyspline::BSpline spline;
		tinyspline::BSpline spline_derive;
		tinyspline::BSpline spline_derive_sq;
		cv::Point3f eval_spline_Point3f(tinyspline::BSpline* sp, float index);
};

#endif
