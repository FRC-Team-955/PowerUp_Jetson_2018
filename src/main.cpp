#include <field_dimensions.h>
#include <field_renderer.h>
#include <multi_waypoint_calculator.h>
#include <opencv2/opencv.hpp>
#include <renderer.h>
#include <socket.h>

#define JUST_RENDER false

namespace FD = FieldDimension;
namespace MM = MiscMath;

float random_float() { return ((double)rand() / (RAND_MAX)); }

// TODO:
//    Make creating waypoints cleaner/easier
//    Documentation
//    Proper error reporting and logging; Return enum with error code instead of bool? 
//    	0 for success, any for fail for partial backwards compat.
//    Action items, attached to waypoints (Executes at end of path)

int main() {
#if JUST_RENDER
	Renderer::init();
#else
	SocketServer sock(5801);
#endif

	srand(time(NULL));
	const float wheel_width = 660.0;

	MultiWaypointCalculator path(wheel_width / 2.0, 20.0);

	// Position, Start Velocity End Velocity, Direction (rel. to origin), outcrop
	
	//Start
	path.reset_and_begin(
			{cv::Point2f(1000.0, 1000.0),
			0.25, 1.0, MM::pi / 2.0f, wheel_width});
	
	//Switch front
	path.push_back(
			{FD::Switch::front_center_left - cv::Point2f(0.0, wheel_width / 2.0f),
			0.25, 1.0, MM::pi / 2.0f, wheel_width}, false);

	//Back up
	path.push_back(
			{cv::Point2f(1000.0, 2000.0),
			0.25, 1.0, MM::pi / 2.0f, wheel_width}, true);

	//Trek forward
	path.push_back(
			{cv::Point2f(1000.0, 7000.0),
			1.0, 1.0, MM::pi / 2.0f, wheel_width}, false);

	//Meet the other side of the switch
	path.push_back(
			{FD::Switch::back_center_left + cv::Point2f(0.0, wheel_width / 2.0f),
			0.25, 1.0, (3.0f * MM::pi) / 2.0f, wheel_width}, false);

	TankDriveCalculator::TankOutput output;
	while (path.evaluate(output)) {
#if JUST_RENDER
		Renderer::clear();
		Renderer::bound(FD::field_bounds, 4.0);
		Renderer::grid(1000.0, 1000.0, 0.2, 0.2, 0.2, FD::field_bounds);
		FieldRenderer::render((char *)"RL", false);

		path.render();
		Renderer::display();
	}
#else
		bool abort;
		sock.write_to(&output.motion, sizeof(TankDriveCalculator::TankOutput));
		sock.read_to(&abort, sizeof(bool)); // Read once before we update the spline
	}
	output.motion.delta_time = 0.0;
	while (true) {
		bool abort;
		sock.write_to(&output.motion, sizeof(TankDriveCalculator::TankOutput));
		sock.read_to(&abort, sizeof(bool)); // Read once before we update the spline
	}
#endif
}
