#ifndef SINE_WAVE_CALCULATOR_H
#define SINE_WAVE_CALCULATOR_H

#include <opencv2/opencv.hpp>
#include <tank_drive_calculator.h>
#include <sine_wrap.h>
#include <vector>
#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <renderer.h>

class SineWaveCalculator {
	public:
		SineWaveCalculator (
		cv::Point2f position_start,
		float amplitude,
		float period,
		float max_change_time,
		float wheel_distance,
		size_t repeats);
		~SineWaveCalculator();

		float max_change_time;
		float min_velocity;
		float wheel_distance;
		void render();

		TankDriveCalculator::TankOutput evaluate(bool advance);

	private:
		SineWrap* wrap;

};

#endif
