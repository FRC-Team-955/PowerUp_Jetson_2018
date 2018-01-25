#ifndef MULTI_WAYPOINT_CALCULATOR_H
#define MULTI_WAYPOINT_CALCULATOR_H

#include <waypoint.h>
#include <tank_drive_calculator.h>
#include <spline_wrap.h>

class MultiWaypointCalculator : SQDerivable {
	public:
		MultiWaypointCalculator(std::vector<WayPoint> path);		
		void render();

		//void replace_current_start(WayPoint surrogate);
	private:
		float wheel_distance;
		float max_change_time;

		std::vector<WayPoint> path;
		std::vector<SplineWrap> splines;

		void rebuild_splines();
	

};

#endif
