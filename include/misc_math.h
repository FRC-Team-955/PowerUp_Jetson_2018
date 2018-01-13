#ifndef MISC_MATH_H
#define MISC_MATH_H
#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>
namespace MiscMath {
	bool ToleranceCheck(float input, float expect, float tolerance);

	float PointDistance(cv::Point* a, cv::Point* b);

	float PointDistance(cv::Point a, cv::Point b);

	float PointDistance(cv::Point2f a, cv::Point2f b);

	float NegativeReciprocal(float slope);

	cv::Point2f MoveAlongLine(bool forward, float distance, float slope, cv::Point2f start);

	cv::Point MidPoint(cv::Point* a, cv::Point* b);

	cv::Point MidPoint(cv::Point a, cv::Point b);

	cv::Point GetCenter(cv::Rect* rectangle);

	cv::Point2f RadialOffset(float radians, float distance, cv::Point2f offset);

	float LineSlope(cv::Point2f a, cv::Point2f b);
	}
#endif
