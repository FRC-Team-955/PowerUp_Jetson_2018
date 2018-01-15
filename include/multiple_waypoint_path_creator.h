#ifndef MULTIPLE_WAYPOINT_PATH_CREATOR_H
#define MULTIPLE_WAYPOINT_PATH_CREATOR_H
#include <vector>
#include <path_calculator.h>

class MultipleWaypointPathCreator {
	public:
		MultipleWaypointPathCreator(int nCtrlp, float max_time_step, float wheel_distance);
		void clear_points();
		bool push_point_direction(cv::Point2f position, float direction, float speed_start, float speed_end);
		Path calculate_path();
	private:
		size_t spline_iter = 0;
		size_t max_ctrlp = 0;
		tinyspline::BSpline spline;
		std::vector<tinyspline::real> ctrlp;
		float max_time_step; 
		float wheel_distance;
};
#endif
