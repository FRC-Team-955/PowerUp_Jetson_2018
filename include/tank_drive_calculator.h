#ifndef TANK_DRIVE_CALCULATOR_H
#define TANK_DRIVE_CALCULATOR_H

#include <misc_math.h>
#include <opencv2/opencv.hpp>
#include <shared_network_types.h>
#include <sq_derivable.h>
#include <renderer.h>

#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <memory>

namespace MM = MiscMath;

class TankDriveCalculator {
	public:
		TankDriveCalculator(
				std::shared_ptr<SQDerivable> function,
				float wheel_distance,
				float time_step)
			: function(function), wheel_distance(wheel_distance),
			time_step(time_step) {
				index = function->start_index;
			}
		struct TankOutput {
			TankDriveMotionUnit motion;
			cv::Point2f left_position;
			cv::Point2f right_position;
			cv::Point2f center_position;
			float robot_direction;
		};

		bool evaluate(TankOutput& output, bool advance);
		void render();
		void render_robot();

	private:
		float index = 0.0;
		bool evaluate(TankOutput& output, float& index, bool advance);
		std::shared_ptr<SQDerivable> function; // TODO: Make this useful with any SQ_Derivable
		float wheel_distance;
		float time_step;
};

#endif
