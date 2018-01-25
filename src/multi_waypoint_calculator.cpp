#include <multi_waypoint_calculator.h>

MultiWaypointCalculator::MultiWaypointCalculator(std::vector<WayPoint> path)
	: path(path) {
		rebuild_splines();
	}

void MultiWaypointCalculator::rebuild_splines() {
	min_index = 0.0;
	max_index = 0.0;
	if (path.size() > 1) {
		for (size_t i = 1; i < path.size(); i++) {
			splines.push_back(SplineWrap(path[i - 1], path[i]));
			max_index += splines.back().max_index;
		}
	} else {
		std::cerr << "Cannot build path from one waypoint" << std::endl;
	}
}

void evaluate(float index) {
	
}
