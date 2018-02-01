#include <multi_waypoint_calculator.h>

void MultiWaypointCalculator::reset_and_begin(WayPoint input) {
	path.clear();
	beginning = input;
}

bool MultiWaypointCalculator::evaluate(TankDriveCalculator::TankOutput &output) {
		//Get to the next valid path, skipping immediately void ones.
		while (path.size() && !path.back().path.evaluate(output, true)) {
			path.pop_back();
		}
		return path.size() > 0;
}

bool MultiWaypointCalculator::replace_current(WayPoint input) {
	if (path.size() > 0) {
		WayPoint end = path.back().end;
		bool reverse = path.back().reverse;
		TankDriveCalculator calc (
			std::make_shared<SplineWrap> (input, end),
			wheel_distance,
			max_change_time,
			reverse );
		path.pop_back();	
		path.push_back( { calc, end, reverse } );
		return true;
	} else {
		return false;
	}
}

bool MultiWaypointCalculator::push_back(WayPoint input, bool reverse, bool back) {
	WayPoint start; 
	if (path.size() == 0) {
		if (beginning) {
			start = beginning.value();
		} else {
			return false;
		}
	} else {
		start = path.front().end;
	}

	TankDriveCalculator calc (
			std::make_shared<SplineWrap> (start, input),
			wheel_distance,
			max_change_time,
			reverse );

	path.push_front( { calc, input, reverse } );
	return true;
}

void MultiWaypointCalculator::render() {
	if (path.size() > 0) {
		for (auto& path_render : path)
			path_render.path.render();
		path.back().path.render_robot();
	}
}
