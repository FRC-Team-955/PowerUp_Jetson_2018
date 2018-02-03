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
		void reset_and_begin(WayPoint input);
		bool replace_current(WayPoint input);
		bool push_back(WayPoint input, bool reverse);
		bool evaluate(TankDriveCalculator::TankOutput &output);

	private:
		float wheel_distance;
		float max_change_time;

		// A pair of the current path and it's endpoint, because it's
		// not stored in the object itself.
		struct PathEndPair {
			TankDriveCalculator path;
			WayPoint end;
			bool reverse;
		};

		// The seed beginning of the path, because a path cannot be constructed from
		// a single point
		// Also: Habits die hard, Rust has spoiled me...
		std::optional<WayPoint> beginning;

		// Back is the current path, front is the paths in the future
		std::deque<PathEndPair> path;
};

#endif
