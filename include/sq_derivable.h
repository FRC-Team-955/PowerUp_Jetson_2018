#ifndef SQ_DERIVABLE_H
#define SQ_DERIVABLE_H

#include <opencv2/opencv.hpp>

class SQDerivable {
	public:
		virtual cv::Point2f evaluate_position(); //Get x(index), y(index)
		virtual cv::Point2f evaluate_velocity_xy(); //Get x'(index), y'(index)
		virtual cv::Point2f evaluate_acceleration_xy(); //Get x''(index), y''(index)
		virtual bool advance(float index); //Advance index by amount, returns true if successful
		float evaluate_velocity_magnitude() {
			return cv::norm(evaluate_velocity_xy());
		}
		float evaluate_direction() {
			cv::Point2f velocity = evaluate_velocity_xy();
			return std::atan2(velocity.y, velocity.x);
		};
	private:
		float index;
};

#endif
