#ifndef MISC_MATH_H
#define MISC_MATH_H
#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>
namespace MiscMath {
	bool ToleranceCheck(float input, float expect, float tolerance);

	float PointDistance(cv::Point2f a, cv::Point2f b);

	float PointDistance(cv::Point2f a, cv::Point2f b);

	float NegativeReciprocal(float slope);

	cv::Point2f MoveAlongLine(bool forward, float distance, float slope, cv::Point2f start);

	//Takes the normal to a vector (I.E rotates it 90 degrees upward)
	cv::Point2f NormalTo(cv::Point2f input);

	cv::Point2f MidPoint(cv::Point2f a, cv::Point2f b);

	cv::Point2f RadialOffset(float radians, float distance, cv::Point2f offset);

	float LineSlope(cv::Point2f a, cv::Point2f b);

	cv::Point2f From3f_xy(cv::Point3f point);

	cv::Point3f From2f_xy(cv::Point2f point, float z);

	const float pi = std::acos(-1);
}
#endif
