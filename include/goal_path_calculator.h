#ifndef GOAL_PATH_CALCULATOR_H
#define GOAL_PATH_CALCULATOR_H

#include <vector>
#include <path_calculator.h>

namespace GoalPathCalculator {
	extern Path calculate_path(
			float direction_start,
			cv::Point2f position_start,
			float direction_end,
			cv::Point2f position_end,
			float wheel_distance,
			float max_allowed_velocity,
			float max_time_step,
			float min_velocity
			);
}
#endif
