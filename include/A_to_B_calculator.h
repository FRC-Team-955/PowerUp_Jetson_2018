#ifndef A_TO_B_CALCULATOR
#define A_TO_B_CALCULATOR

#include <opencv2/opencv.hpp>
#include <tank_drive_calculator.h>
#include <spline_wrap.h>
#include <vector>
#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <renderer.h>

class AToB {
	public:
		AToB (float direction_start,
				cv::Point2f position_start,
				float direction_end,
				cv::Point2f position_end,
				float wheel_distance,
				float max_allowed_velocity,
				float max_change_time,
				float min_velocity);
		~AToB();

		float max_allowed_velocity;
		float max_change_time;
		float min_velocity;
		float wheel_distance;
		bool reverse = false;
		void render();

		TankDriveCalculator::TankOutput evaluate(bool advance);	

	private:
		SplineWrap* wrap;

};

#endif
