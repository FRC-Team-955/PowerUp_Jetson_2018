#include <path_calculator.h>
Path::Path(tinyspline::BSpline spline, float wheel_distance, float max_change_time, bool reverse) {
	this->spline = spline;
	this->spline_derive = this->spline.derive();
	this->spline_derive_sq = this->spline_derive.derive();
	this->wheel_distance = wheel_distance;
	this->max_change_time = max_change_time;
	this->reverse = reverse;
}

bool Path::next_point (TankDriveMotionUnit* output) { //TODO: Optionally return these coords or display them
	cv::Point2f dummy_left;
	cv::Point2f dummy_right;
	return next_point_raw(output, &this->official_traversal, &dummy_left, &dummy_right);
}

void Path::get_current_loc_nondestruct(cv::Point2f* out_center, float* out_angle) {
		auto point_sp = spline.evaluate(official_traversal.spline_index).result();
		auto point_dr = spline_derive.evaluate(official_traversal.spline_index).result();
		*out_center = cv::Point2f(point_sp[0], point_sp[1]);
		cv::Point2f point_dr_cv = cv::Point2f(point_dr[0], point_dr[1]);
		*out_angle = atan2(point_dr_cv.y, point_dr_cv.x);
}

bool Path::next_point_raw (TankDriveMotionUnit* output, Traversal* traversal, cv::Point2f* out_left, cv::Point2f* out_right) {
		//Calculate spline evaluations
		auto point_sp = spline.evaluate(traversal->spline_index).result();
		auto point_dr = spline_derive.evaluate(traversal->spline_index).result();
		auto point_dr_sq = spline_derive_sq.evaluate(traversal->spline_index).result();
		cv::Point2f point_sp_cv = cv::Point2f(point_sp[0], point_sp[1]);
		cv::Point2f point_dr_cv = cv::Point2f(point_dr[0], point_dr[1]);
		cv::Point2f point_dr_sq_cv = cv::Point2f(point_dr_sq[0], point_dr_sq[1]);

		//We hijack the third axis (Z) to use it as a velocity max set point. heh.
		float max_allowed_velocity = point_sp[2];

		//How much change in dist do we expect?
		float sum_dr_squares = powf(point_dr_cv.x, 2.0) + powf(point_dr_cv.y, 2.0); //TODO: Just use multiplication here instead of powf?
		float speed_center = cv::norm(point_dr_cv); //Distance between this point and the next

		//Create paths for each wheel
		cv::Point2f point_norm_raw_cv = cv::Point2f(-point_dr_cv.y, point_dr_cv.x);
		cv::Point2f point_norm_raw_sq_cv = cv::Point2f(-point_dr_sq_cv.y, point_dr_sq_cv.x);
		cv::Point2f point_norm_cv = (point_norm_raw_cv / speed_center) * wheel_distance;

		//Rate of travel in center over i
		float dr_speed_center = ((point_dr_sq_cv.y * point_dr_cv.x) + (point_dr_sq_cv.x * point_dr_cv.y)) / speed_center;

		//How much should we move the derivative from the original
		cv::Point2f dr_offset_speed = (wheel_distance * ((point_norm_raw_sq_cv * speed_center) - (point_norm_raw_cv * dr_speed_center)) / sum_dr_squares);

		//Calculate left speed with some calculus
		cv::Point2f dr_left = point_dr_cv + dr_offset_speed; 
		float speed_left = cv::norm(dr_left);

		//Calculate right speed with some more calculus
		cv::Point2f dr_right = point_dr_cv - dr_offset_speed;
		float speed_right = cv::norm(dr_right);
		
		//The max of the two is the farthest distance
		float speed_max = std::max(speed_left, speed_right);

		//Find change in angle
		double change_in_slope = ((point_dr_sq_cv.y*point_dr_cv.x) - (point_dr_sq_cv.x*point_dr_cv.y)) / powf(point_dr_cv.x, 2.0);
		double change_in_angle = (1.0 / (1.0 + powf(point_dr_cv.y / point_dr_cv.x, 2.0))) * change_in_slope;
		float reverse_left = change_in_angle > pi * 2.0 ? -1.0 : 1.0;
		float reverse_right = -change_in_angle > pi * 2.0 ? -1.0 : 1.0;

		float absolute_velocity_left = max_allowed_velocity * (speed_left / speed_max) * reverse_left; 
		float absolute_velocity_right = max_allowed_velocity * (speed_right / speed_max) * reverse_right; 

		//Accumulate distances, assuming the longest side always goes one distance unit
		traversal->left_accum += absolute_velocity_left * max_change_time;
		traversal->right_accum += absolute_velocity_right * max_change_time;
		
		TankDriveMotionUnit::Special special = TankDriveMotionUnit::Special::Middle;
		if (traversal->spline_index == 0.0) {
			special = TankDriveMotionUnit::Special::Beginning;
		}

		traversal->spline_index += (1.0 / speed_max) * max_allowed_velocity * max_change_time;

		if (traversal->spline_index >= 1.0) {
			special = TankDriveMotionUnit::Special::End;
		}

		output->position_left = reverse ? -traversal->left_accum : traversal->left_accum;
		output->velocity_left = reverse ? -absolute_velocity_left : absolute_velocity_left;
		output->position_right = reverse ? -traversal->right_accum : traversal->right_accum;
		output->velocity_right = reverse ? -absolute_velocity_right : absolute_velocity_right;
		output->delta_time = max_change_time;
		output->special = special;
		*out_left = point_sp_cv + point_norm_cv;
		*out_right = point_sp_cv - point_norm_cv;
		return traversal->spline_index < 1.0; //Allow further reads if we're not too far
}


void Path::render()
{
	Traversal traversal;
	TankDriveMotionUnit current_talonpoint;
	cv::Point2f last_left;
	cv::Point2f last_right;
	cv::Point2f current_left;
	cv::Point2f current_right;
	glColor3f(0.0, 0.0, 0.0);
	glPointSize(9);
	glBegin(GL_LINES);

	next_point_raw(&current_talonpoint, &traversal, &current_left, &current_right);
	last_left = current_left;
	last_right = current_right;

	while(next_point_raw(&current_talonpoint, &traversal, &current_left, &current_right)) {
		Renderer::color_by(current_talonpoint.velocity_left);
		glVertex2f(last_left.x, last_left.y);
		glVertex2f(current_left.x, current_left.y);

		Renderer::color_by(current_talonpoint.velocity_right);
		glVertex2f(last_right.x, last_right.y);
		glVertex2f(current_right.x, current_right.y);
		last_left = current_left;
		last_right = current_right;
	}

	glEnd();
}
