#ifndef MULTI_WAYPOINT_CALCULATOR_H
#define MULTI_WAYPOINT_CALCULATOR_H

#include <waypoint.h>
#include <tank_drive_calculator.h>
#include <spline_wrap.h>

class MultiWaypointCalculator : SQDerivable {
	public:
		MultiWaypointCalculator();		
		void render();

	private:
		float wheel_distance;
		float max_change_time;

		std::vector<WayPoint> path;
		std::vector<SplineWrap> splines;

		void rebuild_splines();
	

};

#endif
