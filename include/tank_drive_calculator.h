#ifndef TANK_DRIVE_CALCULATOR_H
#define TANK_DRIVE_CALCULATOR_H

#include <opencv2/opencv.hpp>
#include <shared_network_types.h>
#include <sq_derivable.h>
#include <misc_math.h>

namespace TankDriveCalculator {

	struct TankOutput {
		TankDriveMotionUnit motion;
		cv::Point3f left_position;
		cv::Point3f right_position;
		cv::Point2d center_position;
		float robot_direction;
	};

	TankOutput evaluate (SQDerivable* function, float wheel_distance, float max_change_time, bool reverse, bool advance, float *index);
}

#endif
