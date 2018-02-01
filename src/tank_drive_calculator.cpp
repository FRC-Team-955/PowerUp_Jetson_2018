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
		output.motion.velocity_left = (dj * dp_dj_left) / time_step;
		output.motion.velocity_right = (dj * dp_dj_right) / time_step;

		//Should we completely reverse the bot?
		if (reverse) {
			output.motion.velocity_left *= -1.0;
			output.motion.velocity_right *= -1.0;
		}

		output.motion.delta_time = time_step;

		output.robot_direction = function->direction_xy();

		output.motion.position_left = 0.0;
		output.motion.position_right = 0.0;

		index += fabs(dj);

		//Hard turns (Where one motor's velocity more than doubles the other's velocity) need to reverse one motor
		if (((dp_dj_left - function->velocity_magnitude_xy()) / wheel_distance > 1.0) &&
				function-> change_in_angle() > 0.0)
			output.motion.velocity_left *= -1.0;

		if (((dp_dj_right - function->velocity_magnitude_xy()) / wheel_distance > 1.0) &&
				function-> change_in_angle() < 0.0)
			output.motion.velocity_right *= -1.0;

		return true;
	} else {
		return false;
	}
}

void TankDriveCalculator::render() {
	float render_index = 0.0;
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
	TankDriveCalculator::TankOutput output;
	evaluate(output, false);
	glColor3f(0.8, 0.8, 0.8);
	glLineWidth(3);
	std::vector<cv::Point2f> wireframe;
	wireframe.push_back(cv::Point2f ((wheel_distance), (wheel_distance)));
	wireframe.push_back(cv::Point2f (-(wheel_distance), (wheel_distance)));
	wireframe.push_back(cv::Point2f (-(wheel_distance), -(wheel_distance)));
	wireframe.push_back(cv::Point2f ((wheel_distance), -(wheel_distance)));

	cv::Mat rot_mat( 2, 3, CV_32FC1 );
	rot_mat = cv::getRotationMatrix2D(cv::Point2f(0.0, 0.0), -function->direction_xy() * (180.0 / acos(-1)), 1.0);
	cv::transform(wireframe, wireframe, rot_mat);

	cv::Point2f last = wireframe.back();
	glBegin(GL_LINES);
	for (auto& point : wireframe) {
		glVertex2f(point.x + function->position.x, point.y + function->position.y);
		glVertex2f(last.x + function->position.x, last.y + function->position.y);
		last = point;
	}
	glVertex2f(function->position.x, function->position.y);
	if (reverse) {
		glVertex2f(function->position.x - (cos(function->direction_xy()) * (wheel_distance)), function->position.y - (sin(function->direction_xy()) * (wheel_distance)));
	} else {
		glVertex2f(function->position.x + (cos(function->direction_xy()) * (wheel_distance)), function->position.y + (sin(function->direction_xy()) * (wheel_distance)));
	}
	glEnd();
}
