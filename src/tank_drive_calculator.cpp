#include <tank_drive_calculator.h>

TankDriveCalculator::TankOutput TankDriveCalculator::evaluate (SQDerivable* function, float wheel_distance, float max_change_time, bool reverse) {
		TankOutput output;

		//We hijack the third axis (Z) to use it as a velocity max set point. heh.
		float max_allowed_velocity = function->position.z;

		//How much change in dist do we expect?
		float sum_dr_squares = powf(function->velocity.x, 2.0) + powf(function->velocity.y, 2.0); //TODO: Just use multiplication here instead of powf?
		float speed_center = cv::norm(function->velocity); //Distance between this point and the next

		//Create paths for each wheel
		cv::Point2f point_norm_raw_cv = MiscMath::NormalTo(MiscMath::From3f_xy(function->velocity));
		cv::Point2f point_norm_raw_sq_cv = MiscMath::NormalTo(MiscMath::From3f_xy(function->acceleration));
		cv::Point2f point_norm_cv = (point_norm_raw_cv / speed_center) * wheel_distance;

		//Rate of travel in center over i
		float dr_speed_center = ((function->acceleration.y * function->velocity.x) + (function->acceleration.x * function->velocity.y)) / speed_center;

		//How much should we move the derivative from the original
		cv::Point2f dr_offset_speed = (wheel_distance * ((point_norm_raw_sq_cv * speed_center) - (point_norm_raw_cv * dr_speed_center)) / sum_dr_squares);

		//Calculate left speed with some calculus
		cv::Point2f dr_left = MiscMath::From3f_xy(function->velocity) + dr_offset_speed; 
		float speed_left = cv::norm(dr_left);

		//Calculate right speed with some more calculus
		cv::Point2f dr_right = MiscMath::From3f_xy(function->velocity) - dr_offset_speed;
		float speed_right = cv::norm(dr_right);
		std::cout << "P " << function->position << std::endl;
		std::cout << "V " << function->velocity << std::endl;
		std::cout << "A " << function->acceleration << std::endl;

		//The max of the two is the farthest distance
		float speed_max = std::max(speed_left, speed_right);

		//Find change in angle
		double change_in_slope = ((function->acceleration.y*function->velocity.x) - (function->acceleration.x*function->velocity.y)) / powf(function->velocity.x, 2.0);
		double change_in_angle = (1.0 / (1.0 + powf(function->velocity.y / function->velocity.x, 2.0))) * change_in_slope;
		float reverse_left = change_in_angle > MiscMath::pi * 2.0 ? -1.0 : 1.0;
		float reverse_right = -change_in_angle > MiscMath::pi * 2.0 ? -1.0 : 1.0;

		float absolute_velocity_left = max_allowed_velocity * (speed_left / speed_max) * reverse_left; 
		float absolute_velocity_right = max_allowed_velocity * (speed_right / speed_max) * reverse_right; 

		//Accumulate distances, assuming the longest side always goes one distance unit
		//traversal->left_accum += absolute_velocity_left * max_change_time;
		//traversal->right_accum += absolute_velocity_right * max_change_time;

		TankDriveMotionUnit::Special special = TankDriveMotionUnit::Special::Middle;
		if (function->is_at_beginning()) {
			special = TankDriveMotionUnit::Special::Beginning;
		}

		function->advance((1.0 / speed_max) * max_allowed_velocity * max_change_time);

		if (function->is_at_end()) {
			special = TankDriveMotionUnit::Special::End;
		}

		output.motion.position_left = 0.0;//reverse ? -traversal->left_accum : traversal->left_accum;
		output.motion.velocity_left = reverse ? -absolute_velocity_left : absolute_velocity_left;
		output.motion.position_right = 0.0;//reverse ? -traversal->right_accum : traversal->right_accum;
		output.motion.velocity_right = reverse ? -absolute_velocity_right : absolute_velocity_right;
		output.motion.delta_time = max_change_time;
		output.motion.special = special;
		output.left_position = function->position + MiscMath::From2f_xy(point_norm_cv, 0.0);
		output.right_position = function->position - MiscMath::From2f_xy(point_norm_cv, 0.0);
		output.center_position = MiscMath::From3f_xy(function->position);
		output.robot_direction = function->evaluate_direction_xy();
		return output; //Allow further reads if we're not too far
	}
