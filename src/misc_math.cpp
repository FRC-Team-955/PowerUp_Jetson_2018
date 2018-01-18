#include <misc_math.h>

bool MiscMath::ToleranceCheck(float input, float expect, float tolerance)
{
    return fabs(input - expect) <= tolerance;
}

float MiscMath::PointDistance(cv::Point2f a, cv::Point2f b)
{
    return cv::norm(b - a);
}

float MiscMath::NegativeReciprocal(float slope)
{
    return -1.0f / slope;
}

cv::Point2f MiscMath::MoveAlongLine(bool forward, float distance, float slope, cv::Point2f start)
{
    float k = (forward ? distance : -distance) / sqrtf(1 + pow(slope, 2));
    return start + (k * cv::Point2f(1, slope));
}

cv::Point2f MiscMath::MidPoint(cv::Point2f a, cv::Point2f b)
{
    return (a + b) / 2;
}

cv::Point2f MiscMath::RadialOffset(float radians, float distance, cv::Point2f offset)
{
	return cv::Point2f(
			(std::cos(radians) * distance) + offset.x,
			(std::sin(radians) * distance) + offset.y);
}

float MiscMath::LineSlope(cv::Point2f a, cv::Point2f b) {
	return (b.y - a.y) / (b.x - a.x);
}

cv::Point2f MiscMath::NormalTo(cv::Point2f input) {
	return cv::Point2f(-input.y, input.x);
}

cv::Point2f MiscMath::From3f_xy(cv::Point3f point) {
	return cv::Point2f(point.x, point.y);
}

cv::Point3f MiscMath::From2f_xy(cv::Point2f point, float z) {
	return cv::Point3f(point.x, point.y, z);
}
