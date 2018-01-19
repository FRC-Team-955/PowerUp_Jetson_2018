#include <sine_wave_calculator.h>

SineWaveCalculator::SineWaveCalculator (
		cv::Point2f position_start,
		float amplitude,
		float period,
		float max_change_time,
		float wheel_distance,
		size_t repeats) {
	wrap = new SineWrap(position_start, amplitude, period, repeats);

	this->max_change_time = max_change_time;
	this->wheel_distance = wheel_distance;
}

TankDriveCalculator::TankOutput SineWaveCalculator::evaluate(bool advance) {
	return TankDriveCalculator::evaluate(wrap, wheel_distance, max_change_time, false, advance);
}

void SineWaveCalculator::render()
{
	float temp_copy = wrap->function_index;
	wrap->function_index = 0.0;
	wrap->advance(0.0);
	cv::Point3f last_left;
	cv::Point3f last_right;
	TankDriveCalculator::TankOutput output = evaluate(true);
	glColor3f(0.0, 0.0, 0.0);
	glPointSize(9);
	glBegin(GL_LINES);

	last_left = output.left_position;
	last_right = output.right_position;

	while(true) {
		TankDriveCalculator::TankOutput output = evaluate(true);
		Renderer::color_by(output.motion.velocity_left);
		glVertex2f(last_left.x, last_left.y);
		glVertex2f(output.left_position.x, output.left_position.y);

		Renderer::color_by(output.motion.velocity_right);
		glVertex2f(last_right.x, last_right.y);
		glVertex2f(output.right_position.x, output.right_position.y);
		last_left = output.left_position;
		last_right = output.right_position;
		if (output.motion.special == TankDriveMotionUnit::Special::End)
			break; //TODO: Add timeout too!
	}
	glEnd();

	wrap->function_index = temp_copy;
	wrap->advance(0.0);
}

SineWaveCalculator::~SineWaveCalculator() {
	delete wrap;
}
