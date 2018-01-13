#include <goal_path_calculator.h>
namespace GoalPathCalculator {
	Path calculate_path(
			float direction_start,
			cv::Point2f position_start,
			float direction_end,
			cv::Point2f position_end,
			float wheel_distance,
			float max_allowed_velocity,
			float max_time_step,
			float min_velocity
			) {
		tinyspline::BSpline spline (6, 3, 3);
		std::vector<tinyspline::real> ctrlp = spline.ctrlp();
		ctrlp[0] = position_start.x;
		ctrlp[1] = position_start.y;
		ctrlp[2] = min_velocity;

		auto leading_a_start = MiscMath::RadialOffset(direction_start, wheel_distance, position_start);
		ctrlp[3] = leading_a_start.x;
		ctrlp[4] = leading_a_start.y;
		ctrlp[5] = max_allowed_velocity;

		auto leading_b_start = MiscMath::RadialOffset(direction_start, wheel_distance * 2.0, position_start);
		ctrlp[6] = leading_b_start.x;
		ctrlp[7] = leading_b_start.y;
		ctrlp[8] = max_allowed_velocity;

		auto leading_a_end = MiscMath::RadialOffset(direction_end, wheel_distance * 2.0, position_end);
		ctrlp[9] = leading_a_end.x;
		ctrlp[10] = leading_a_end.y;
		ctrlp[11] = max_allowed_velocity;

		auto leading_b_end = MiscMath::RadialOffset(direction_end, wheel_distance, position_end);
		ctrlp[12] = leading_b_end.x;
		ctrlp[13] = leading_b_end.y;
		ctrlp[14] = max_allowed_velocity;

		ctrlp[15] = position_end.x;
		ctrlp[16] = position_end.y;
		ctrlp[17] = min_velocity;

		spline.setCtrlp(ctrlp);
		return Path(spline, wheel_distance, max_time_step);
	}
}
