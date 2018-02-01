#ifndef SPIRAL_WRAP_H
#define SPIRAL_WRAP_H

#include <opencv2/opencv.hpp>
#include <sq_derivable.h>
#include <misc_math.h>

class SpiralWrap : public SQDerivable {
	public:
		SpiralWrap(cv::Point2f center, float distance, float max_index);
		bool evaluate(float index);
	private:
		float distance;
		cv::Point2f center;
};

#endif
