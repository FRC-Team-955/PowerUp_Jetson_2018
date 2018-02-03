#include <multi_waypoint_calculator.h>

// Reset the entire path, seeding the beginning
void MultiWaypointCalculator::reset_and_begin(WayPoint input) {
	path.clear();
	beginning = input;
}

// Evaluate the next point on the full path, transitioning smoothly from one point to the next
bool MultiWaypointCalculator::evaluate(TankDriveCalculator::TankOutput &output) {
		// Get to the next valid path, skipping immediately void ones.
		while (path.size() && !path.back().path.evaluate(output, true)) {
			path.pop_back();
		}
		// Do we still have a path to follow?
		return path.size() > 0;
}

// Swap our current path's starting point with the given waypoint, keeping the end point 
//    Mainly used when we need to correct a drift
void MultiWaypointCalculator::replace_current(WayPoint input) {
	if (path.size() > 0) {
		WayPoint end = path.back().end;
		bool reverse = path.back().reverse;

		// Create the new path
		TankDriveCalculator calc (
			std::make_shared<SplineWrap> (input, end, reverse),
			wheel_distance,
			max_change_time);

		path.pop_back(); // Remove the old path

		// Add the new path
		path.push_back( { calc, end, reverse } );
	} else {
		// There's no other end point, just replace the beginning
		beginning_exists = true;
		beginning = input;
	}
}

// Push a new objective to the full path, caching it's end point so we can
// swap it later
void MultiWaypointCalculator::push_back(WayPoint input, bool reverse) {
	// Either use the last spline's end, or the seed start
	WayPoint start; 
	if (path.size() == 0) {
		start = beginning;
	} else {
		start = path.front().end;
	}

	// Create the tank drive calculation object
	TankDriveCalculator calc (
			std::make_shared<SplineWrap> (start, input, reverse),
			wheel_distance,
			max_change_time);

	// Push it as the next objective
	path.push_front( { calc, input, reverse } );
}

void MultiWaypointCalculator::render() {
	if (path.size() > 0) {
		for (auto& path_render : path)
			path_render.path.render();
		//Render the bot at the most recent spot
		path.back().path.render_robot();
	}
}
