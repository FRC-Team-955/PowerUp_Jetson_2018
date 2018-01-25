#include <tank_drive_calculator.h>

TankDriveCalculator::TankOutput TankDriveCalculator::evaluate (SQDerivable* func, float wheel_distance, float max_change_time, bool reverse, bool advance, float *index) {
		TankOutput output;

		func->evaluate(*index);

		//We hijack the third axis (Z) to use it as a velocity max set point. heh.
		float max_allowed_velocity = func->position.z;

		//Position = center + (perpendicular vector * d)
		output.center_position = MM::From3f_xy(func->position);
		output.left_position = MM::From3f_xy(func->position) + (wheel_distance * func->perpendicular_unit_vector_xy());
		output.right_position = MM::From3f_xy(func->position) - (wheel_distance * func->perpendicular_unit_vector_xy());

		float dp_dj_left = cv::norm(MM::From3f_xy(func->velocity) + (wheel_distance * func->perpendicular_unit_vector_derivative_xy()));
		float dp_dj_right = cv::norm(MM::From3f_xy(func->velocity) - (wheel_distance * func->perpendicular_unit_vector_derivative_xy()));

		float largest_dp_dj = std::max(dp_dj_left, dp_dj_right);
		float max_dp = max_allowed_velocity * max_change_time;
		float dj = (max_dp / largest_dp_dj);

		output.motion.velocity_left = (dj * dp_dj_left) / max_change_time;
		output.motion.velocity_right = (dj * dp_dj_right) / max_change_time;
		if (reverse) {
			output.motion.velocity_left *= -1.0;
			output.motion.velocity_right *= -1.0;
		}

		output.motion.delta_time = max_change_time;

		output.robot_direction = func->direction_xy();

		output.motion.position_left = 0.0;
		output.motion.position_right = 0.0;

		if (*index > func->max_index) {
			output.motion.special = TankDriveMotionUnit::Special::Beginning;
		} else {
			output.motion.special = TankDriveMotionUnit::Special::Middle;
		}
		if (advance) {
			if (*index + dj >= func->max_index) {
				output.motion.special = TankDriveMotionUnit::Special::End;
				*index = func->max_index;
			} else {
				*index += dj;
			}
		}

		/*
		//Find change in angle
		double change_in_slope = ((func->acceleration.y*func->velocity.x) - (func->acceleration.x*func->velocity.y)) / powf(func->velocity.x, 2.0);
		double change_in_angle = (1.0 / (1.0 + powf(func->velocity.y / func->velocity.x, 2.0))) * change_in_slope;
		float reverse_left = ((speed_left - func->velocity_magnitude_xy()) / wheel_distance > 1.0) && change_in_angle > 0.0 ? -1.0 : 1.0;
		float reverse_right = ((speed_right - func->velocity_magnitude_xy()) / wheel_distance > 1.0) && change_in_angle < 0.0 ? -1.0 : 1.0;

		float absolute_velocity_left = max_allowed_velocity * (speed_left / speed_max) * reverse_left; 
		float absolute_velocity_right = max_allowed_velocity * (speed_right / speed_max) * reverse_right; 
		*/

		//Accumulate distances, assuming the longest side always goes one distance unit
		//traversal->left_accum += absolute_velocity_left * max_change_time;
		//traversal->right_accum += absolute_velocity_right * max_change_time;

		return output;
}
