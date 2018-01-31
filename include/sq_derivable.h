#ifndef SQ_DERIVABLE_H
#define SQ_DERIVABLE_H

#include <opencv2/opencv.hpp>
#include <misc_math.h>

using namespace MiscMath;

//TODO: Implement caching of evaluations at any index, refresh caches after advance
class SQDerivable {
	public:
		cv::Point3f position; //f(index)
		cv::Point3f velocity; //f'(index)
		cv::Point3f acceleration; //f''(index)

		//returns false if at end
		//Advance index by amount, returns true if successful, and sets position, velocity, etc.
		virtual void evaluate(float index) = 0;

		//The total velocity over dj
		float velocity_magnitude_xy() {
			return cv::norm(From3f_xy(velocity));
		}

		//Derivative of the above (Over dj)
		float velocity_magnitude_derivative_xy () {
			return ((acceleration.y * velocity.y) + (acceleration.x * velocity.x)) / velocity_magnitude_xy();
		}

		//Angle of the tangent line relative to the origin
		float direction_xy() {
			return std::atan2(velocity.y, velocity.x);
		};

		//x'(j)^2 + y'(j)^2
		float sum_derivative_squares_xy () {
			return powf(velocity.x, 2.0) + powf(velocity.y, 2.0);
		}

		//Normalized perpendicular vector to the tangent line
		cv::Point2f perpendicular_unit_vector_xy () {
			return NormalTo(From3f_xy(velocity)) / velocity_magnitude_xy();
		}

		//The change in position of the unit vector above over dj
		cv::Point2f perpendicular_unit_vector_derivative_xy () {
			return ((NormalTo(From3f_xy(acceleration)) * velocity_magnitude_xy()) -
				(NormalTo(From3f_xy(velocity)) * velocity_magnitude_derivative_xy()))
				/ sum_derivative_squares_xy();
		}

		//d(slope)/dj
		float change_in_slope() {
			return ((acceleration.y * velocity.x) - (acceleration.x * velocity.y)) / powf(velocity.x, 2.0);
		}

		//d(tan^-1 (y'(j)/x'(j))) / dj
		float change_in_angle() {
			return (1.0 / (1.0 + powf(velocity.y / velocity.x, 2.0))) * change_in_slope();
		}

		float max_index = 1.0;
		float min_index = 0.0;
};

#endif
