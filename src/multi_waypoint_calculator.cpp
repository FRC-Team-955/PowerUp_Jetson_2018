#include <multi_waypoint_calculator.h>

MultiWaypointCalculator::MultiWaypointCalculator(float wheel_distance, float max_change_time) : wheel_distance(wheel_distance), max_change_time(max_change_time) {
}

void MultiWaypointCalculator::reset_and_begin(WayPoint input) {
	path.clear();
	beginning = input;
}

bool MultiWaypointCalculator::evaluate(TankDriveCalculator::TankOutput &output) {
	if (path.size() > 0) {
		output = path.back().path.evaluate(true);
		if (output.motion.special == TankDriveMotionUnit::Special::End && path.size()) {
			if (path.size() > 0) {
				path.pop_back();
			}
		}
		return true;
	} else {
		return false;
	}
}

void MultiWaypointCalculator::replace_current(WayPoint input) {
	if (path.size() > 0) {
		path.pop_back();	
		push(input, true);
	}
}

void MultiWaypointCalculator::push(WayPoint input, bool back) {
	WayPoint start; 
	if (path.size() == 0) {
		start = beginning;
	} else {
		start = path.back().end;
	}

	auto spline = std::make_shared<SplineWrap> (start, input);
	TankDriveCalculator calc (spline, wheel_distance, max_change_time, false );
	if (back) {
		path.push_back( { calc, input } );
	} else {
		path.push_front( { calc, input } );
	}
}

void MultiWaypointCalculator::render() {
	if (path.size() > 0) {
		for (auto& path_render : path)
			path_render.path.render();
		path.back().path.render_robot();
	}
}
