#ifndef SPLINE_WRAP_H
#define SPLINE_WRAP_H

#include <tinysplinecpp.h>
#include <sq_derivable.h>

class SplineWrap : public SQDerivable {
	public:
		SplineWrap(int nCtrlp) {
		}
	private:
		tinyspline::BSpline spline;
		tinyspline::BSpline spline_derive;
		tinyspline::BSpline spline_derive_sq;
};

#endif
