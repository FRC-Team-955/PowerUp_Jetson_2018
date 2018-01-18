#ifndef SPLINE_WRAP_H
#define SPLINE_WRAP_H

#include <tinysplinecpp.h>
#include <sq_derivable.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <renderer.h>

class SplineWrap : public SQDerivable {
	public:
		SplineWrap(int nCtrlp, std::vector<cv::Point3f> points);
		bool advance(float index); //Advance function_index by amount, returns true if successful
		bool is_at_end(); 
		bool is_at_beginning(); 

		bool set_ctrlpts(std::vector<cv::Point3f> points);

		void render();
	private:
		const size_t spline_dimension = 3;
		tinyspline::BSpline spline;
		tinyspline::BSpline spline_derive;
		tinyspline::BSpline spline_derive_sq;
		cv::Point3f eval_spline_Point3f(tinyspline::BSpline* sp, float index);
};

#endif
