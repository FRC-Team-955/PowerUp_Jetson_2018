#ifndef MULTI_WAYPOINT_CALCULATOR_H
#define MULTI_WAYPOINT_CALCULATOR_H

#include <waypoint.h>
#include <tank_drive_calculator.h>
#include <spline_wrap.h>
#include <memory>

class MultiWaypointCalculator {
	public:
		MultiWaypointCalculator(float wheel_distance, float max_change_time);		
		void render();
		void reset_and_begin (WayPoint input); //Set start, clear the stack.
		void replace_current (WayPoint input);
		void push (WayPoint input, bool back=false);
		bool is_finished();
		bool evaluate(TankDriveCalculator::TankOutput& output);

	private:
		float wheel_distance;
		float max_change_time;

		struct Pair {
			TankDriveCalculator path;
			WayPoint end;
		};

		WayPoint beginning;
		std::deque<Pair> path;

};

#endif
