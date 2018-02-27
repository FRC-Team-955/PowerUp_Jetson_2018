#include <field_dimensions.h>
#include <field_renderer.h>
#include <multi_waypoint_calculator.h>
#include <opencv2/opencv.hpp>
#include <renderer.h>
#include <socket.h>

#define JUST_RENDER false

namespace FD = FieldDimension;
namespace MM = MiscMath;

// TODO:
//    Make creating waypoints cleaner/easier
//    Documentation
//    Proper error reporting and logging; Return enum with error code instead of bool? 
//    	0 for success, any for fail for partial backwards compat.
//    Action items, attached to waypoints (Executes at end of path)

const float wheel_width = 660.0;
int main() {
#if JUST_RENDER
	Renderer::init();
#else
	SocketServer sock(5081);
#endif

	while (true) { 

		MultiWaypointCalculator path(wheel_width / 2.0, 10);

		// Position, Start Velocity End Velocity, Direction (rel. to origin), outcrop
		float min_speed = 0.5;
		float max_speed = 1.0;

		//Start
		path.reset_and_begin(
				{cv::Point2f(1000.0, 1000.0),
				min_speed, max_speed, MM::pi / 2.0f, wheel_width});

		//Switch front
		path.push_back(
				((WayPoint){FD::Switch::front,
				 min_speed, max_speed, MM::pi / 2.0f, wheel_width}).before(wheel_width), false);

		//Back up
		path.push_back(
				{cv::Point2f(1000.0, 2000.0),
				min_speed, max_speed, MM::pi / 2.0f, wheel_width}, true);

		//Trek forward
		path.push_back(
				{cv::Point2f(1000.0, 4000.0),
				max_speed, max_speed, MM::pi / 2.0f, wheel_width}, false);

		//Meet the other side of the switch
		path.push_back(
				((WayPoint){FD::Switch::back,
				 min_speed, max_speed, (3.0f * MM::pi) / 2.0f, wheel_width}).before(wheel_width), false);

		bool abort = false;
		RobotCommand command;
		TankDriveCalculator::TankOutput tank_output;
		std::cout << "Begin main loop" << std::endl;
		while (path.evaluate(tank_output)) {
#if JUST_RENDER
			Renderer::clear();
			Renderer::bound(FD::field_bounds, 4.0);
			Renderer::grid(1000.0, 1000.0, 0.2, 0.2, 0.2, FD::field_bounds);
			FieldRenderer::render((char *)"RL", false);
			//std::cout << output.motion.velocity_left << " : " << output.motion.velocity_right << std::endl;
			//std::cout << output.motion.position_left << " : " << output.motion.position_right << std::endl;

			path.render();
			Renderer::display();
#else
			command.motion = tank_output.motion;
			sock.read_to(&abort, sizeof(bool)); 
			sock.write_to(&command, sizeof(command));
			if (abort) {
				break;
			}
#endif
		}
		if (!abort) {
			std::cout << "Stopping" << std::endl;
			command.motion.delta_time = 0.0;
			sock.read_to(&abort, sizeof(bool)); 
			sock.write_to(&command, sizeof(command));
		}
	}
}
