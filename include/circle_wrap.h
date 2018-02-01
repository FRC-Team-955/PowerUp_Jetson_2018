#ifndef CIRCLE_WRAP_H
#define CIRCLE_WRAP_H

#include <opencv2/opencv.hpp>
#include <sq_derivable.h>
#include <misc_math.h>

class CircleWrap : public SQDerivable {
	public:
		CircleWrap(cv::Point2f center, float y_distance, float x_distance);
		bool evaluate(float index);
	private:
		float y_distance;
		float x_distance;
		cv::Point2f center;
};

#endif
