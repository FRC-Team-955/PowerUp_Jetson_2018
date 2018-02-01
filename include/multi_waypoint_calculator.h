#ifndef MULTI_WAYPOINT_CALCULATOR_H
#define MULTI_WAYPOINT_CALCULATOR_H

#include <memory>
#include <spline_wrap.h>
#include <tank_drive_calculator.h>
#include <waypoint.h>
#include <optional>

class MultiWaypointCalculator {
	public:
		MultiWaypointCalculator(float wheel_distance, float max_change_time)
			: wheel_distance(wheel_distance), max_change_time(max_change_time){};
		void render();
		void reset_and_begin(WayPoint input); // Set start, clear the stack.
		bool replace_current(WayPoint input);
		bool push_back(WayPoint input, bool reverse, bool back = false);
		bool is_finished();
		bool evaluate(TankDriveCalculator::TankOutput &output);

	private:
		float wheel_distance;
		float max_change_time;

		struct Pair {
			TankDriveCalculator path;
			WayPoint end;
			bool reverse;
		};

		std::optional<WayPoint> beginning;

		// Back is the current path, front is the paths in the future
		std::deque<Pair> path;
};

#endif
