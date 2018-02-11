#include <tank_drive_calculator.h>

bool TankDriveCalculator::evaluate(TankDriveCalculator::TankOutput& output, bool advance) {
	return evaluate(output, index, advance);
}

bool TankDriveCalculator::evaluate (TankDriveCalculator::TankOutput& output, float& index, bool advance) {
	if (function->evaluate(index)) {
		//We hijack the third axis (Z) to use it as a velocity max set point. heh.
		float max_allowed_velocity = function->position.z;

		//Position = center + (perpendicular vector * d)
		output.center_position = MM::From3f_xy(function->position);
		output.left_position = MM::From3f_xy(function->position) + (wheel_distance * function->perpendicular_unit_vector_xy());
		output.right_position = MM::From3f_xy(function->position) - (wheel_distance * function->perpendicular_unit_vector_xy());

		//Derivative of the above
		float dp_dj_left = cv::norm(MM::From3f_xy(function->velocity) + (wheel_distance * function->perpendicular_unit_vector_derivative_xy()));
		float dp_dj_right = cv::norm(MM::From3f_xy(function->velocity) - (wheel_distance * function->perpendicular_unit_vector_derivative_xy()));

		//Find dj for the time step
		float largest_dp_dj = std::max(dp_dj_left, dp_dj_right);
		float max_dp = max_allowed_velocity * time_step;
		float dj = (max_dp / largest_dp_dj);

		//Save distances over dt as velocity
		float dp_left = (dj * dp_dj_left);
		float dp_right = (dj * dp_dj_right);

		//Assign velocities
		output.motion.velocity_left = dp_left / time_step;
		output.motion.velocity_right = dp_right / time_step;

		//Increment positions
		output.motion.position_left += dp_left;
		output.motion.position_right += dp_right;

		output.motion.delta_time = time_step;

		//TODO: Make the actual function exposed instead of this garbage?
		output.robot_direction = function->direction_xy();

		//Hard turns (Where one motor's velocity more than doubles the other's velocity over the wheel distance), reverse one motor
		if (((dp_dj_left - function->velocity_magnitude_xy()) / wheel_distance > 1.0) &&
				function-> change_in_angle() > 0.0)
			output.motion.velocity_left *= -1.0;

		if (((dp_dj_right - function->velocity_magnitude_xy()) / wheel_distance > 1.0) &&
				function-> change_in_angle() < 0.0)
			output.motion.velocity_right *= -1.0;

		//Increase the index by the amount we need to move one time unit
		index += dj;

		return true;
	} else {
		return false;
	}
}

void TankDriveCalculator::render() {
	float render_index = function->start_index;
	TankDriveCalculator::TankOutput output;

	evaluate(output, render_index, true);

	glColor3f(0.0, 0.0, 0.0);
	glLineWidth(3);
	glBegin(GL_LINES);

	cv::Point2f last_left = output.left_position;
	cv::Point2f last_right = output.right_position;

	while(evaluate(output, render_index, true)) {
		Renderer::color_by(output.motion.velocity_left);
		glVertex2f(last_left.x, last_left.y);
		glVertex2f(output.left_position.x, output.left_position.y);

		Renderer::color_by(output.motion.velocity_right);
		glVertex2f(last_right.x, last_right.y);
		glVertex2f(output.right_position.x, output.right_position.y);

		last_left = output.left_position;
		last_right = output.right_position;
	}
	glEnd();
}

void TankDriveCalculator::render_robot() {
	function->render_robot(wheel_distance, index);
}
