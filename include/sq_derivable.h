#ifndef SQ_DERIVABLE_H
#define SQ_DERIVABLE_H

#include <opencv2/opencv.hpp>

//TODO: Implement caching of evaluations at any index, refresh caches after advance
class SQDerivable {
	public:
		cv::Point3f position; //f(index)
		cv::Point3f velocity; //f'(index)
		cv::Point3f acceleration; //f''(index)

		//returns false if at end
		virtual bool advance(float index_change) = 0; //Advance index by amount, returns true if successful, and sets position, velocity, etc.

		float evaluate_velocity_magnitude_xy() {
			return cv::norm(cv::Point2f(velocity.x, velocity.y));
		}

		float evaluate_direction_xy() {
			return std::atan2(velocity.y, velocity.x);
		};

		float index;
		float max_index;
		float min_index;
};

#endif
