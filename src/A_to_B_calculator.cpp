#include <A_to_B_calculator.h>

AToB::AToB (float direction_start,
				cv::Point2f position_start,
				float direction_end,
				cv::Point2f position_end,
				float wheel_distance,
				float max_allowed_velocity,
				float max_change_time,
				float min_velocity) {
	reverse = (position_end - position_start).dot(cv::Point2f(cos(direction_start), sin(direction_start))) < 0;

	std::vector<cv::Point3f> input_points;

	input_points.push_back(MiscMath::From2f_xy(position_start, min_velocity));

	auto leading_a_start = MiscMath::RadialOffset(reverse ? direction_start +MiscMath::pi: direction_start, wheel_distance, position_start);
	input_points.push_back(MiscMath::From2f_xy(leading_a_start, max_allowed_velocity));

	auto leading_b_start = MiscMath::RadialOffset(reverse ? direction_start +MiscMath::pi: direction_start, wheel_distance * 2.0, position_start);
	input_points.push_back(MiscMath::From2f_xy(leading_b_start, max_allowed_velocity)); 
	auto leading_a_end = MiscMath::RadialOffset(reverse ? direction_end : direction_end + MiscMath::pi, wheel_distance * 2.0, position_end);
	input_points.push_back(MiscMath::From2f_xy(leading_a_end, max_allowed_velocity));

	auto leading_b_end = MiscMath::RadialOffset(reverse ? direction_end : direction_end + MiscMath::pi, wheel_distance, position_end);
	input_points.push_back(MiscMath::From2f_xy(leading_b_end, max_allowed_velocity));

	input_points.push_back(MiscMath::From2f_xy(position_end, min_velocity));
	wrap = new SplineWrap(6, input_points);

	this->max_allowed_velocity = max_allowed_velocity;
	this->max_change_time = max_change_time;
	this->min_velocity = min_velocity;
	this->wheel_distance = wheel_distance;
}

TankDriveCalculator::TankOutput AToB::evaluate() {
	return TankDriveCalculator::evaluate(wrap, wheel_distance, max_change_time, reverse);
}

void AToB::render()
{
	float temp_copy = wrap->function_index;
	/*
	cv::Point3f last_left;
	cv::Point3f last_right;
	TankDriveCalculator::TankOutput output = evaluate();
	glColor3f(0.0, 0.0, 0.0);
	glPointSize(9);
	glBegin(GL_LINES);

	last_left = output.left_position;
	last_right = output.right_position;

	while(true) {
		TankDriveCalculator::TankOutput output = evaluate();
		Renderer::color_by(output.motion.velocity_left);
		glVertex2f(last_left.x, last_left.y);
		glVertex2f(output.left_position.x, output.left_position.y);

		Renderer::color_by(output.motion.velocity_right);
		glVertex2f(last_right.x, last_right.y);
		glVertex2f(output.right_position.x, output.right_position.y);
		last_left = output.left_position;
		last_right = output.right_position;
		if (output.motion.special == TankDriveMotionUnit::Special::End) {
			break; //TODO: Add timeout too!
		}
	}
	glEnd();
	*/

	wrap->render();
	wrap->function_index = temp_copy;
}

AToB::~AToB() {
	delete wrap;
}
