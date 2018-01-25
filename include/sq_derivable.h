#ifndef SQ_DERIVABLE_H
#define SQ_DERIVABLE_H

#include <opencv2/opencv.hpp>
#include <misc_math.h>

namespace MM = MiscMath;

//TODO: Implement caching of evaluations at any index, refresh caches after advance
class SQDerivable {
	public:
		cv::Point3f position; //f(index)
		cv::Point3f velocity; //f'(index)
		cv::Point3f acceleration; //f''(index)

		//returns false if at end
		//Advance index by amount, returns true if successful, and sets position, velocity, etc.
		bool advance(float change_in_index, float *index) {
			if (*index + change_in_index <= max_index && *index + change_in_index >= min_index) {
				*index += change_in_index;
				evaluate(*index);
				return true;
			} else {
				return false;
			}
		}

		virtual void evaluate(float index) = 0;

		float velocity_magnitude_xy() {
			return cv::norm(MM::From3f_xy(velocity));
		}

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
			return MM::NormalTo(MiscMath::From3f_xy(velocity)) / velocity_magnitude_xy();
		}

		//The position of the unit vector above over dj
		cv::Point2f perpendicular_unit_vector_derivative_xy () {
			return ((MM::NormalTo(MiscMath::From3f_xy(velocity)) * velocity_magnitude_xy()) - 
				(MM::NormalTo(MiscMath::From3f_xy(acceleration)) * velocity_magnitude_derivative_xy())) / sum_derivative_squares_xy();
		}

		//Normalized derivative of the pependicular vector to the tangent line
		//cv::Poin2f perpendicular_vector_derivative_xy () {
		//}

		float max_index;
		float min_index;
};

#endif
