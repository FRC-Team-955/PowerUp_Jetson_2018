#include <renderer.h>
#include <opencv2/opencv.hpp>
#include <socket.h>
#include <field_renderer.h>
#include <field_dimensions.h>
#include <multi_waypoint_calculator.h>
/*
#include <tank_drive_calculator.h>
#include <spline_wrap.h>
#include <circle_wrap.h>
#include <spiral_wrap.h>
*/

#define JUST_RENDER true

namespace FD = FieldDimension;
namespace MM = MiscMath;

float random_float () {
	return ((double) rand() / (RAND_MAX));
}

int main () {
#if JUST_RENDER
	Renderer::init();
#else
	SocketServer sock(5801);
#endif

   srand(time(NULL));
	const float wheel_width = 660.0;

	MultiWaypointCalculator path (wheel_width / 2.0, 20.0);

	while (true) {
		path.reset_and_begin({ 
				FD::Switch::front_center_left - cv::Point2f(0.0, wheel_width / 2.0f), 
				0.25, 1.0, MM::pi / 2.0f, wheel_width });
		for (int i = 0; i < 3; i++) {
			path.push_back( {cv::Point2f(1000.0 + random_float() * (FD::field_width - 1000.0), 1000.0 + random_float() * (FD::field_height - 2000.0)), 
					1.0, 1.0, random_float() * MM::pi * 2.0f, wheel_width }, true);
		}

		TankDriveCalculator::TankOutput output;
#if JUST_RENDER
		size_t randspot = random_float() * 200.0;
		size_t idx = 0;
		while (path.evaluate(output)) {
			Renderer::clear();
			Renderer::bound(FD::field_bounds, 4.0);
			Renderer::grid(1000.0, 1000.0, 0.2, 0.2, 0.2, FD::field_bounds);
			FieldRenderer::render((char*)"RL", false);

			if (idx == randspot) {
				path.replace_current({ output.center_position + cv::Point2f(random_float(), random_float()) * 10.0, 
						1.0, 1.0, output.robot_direction + (random_float() * 0.001f), wheel_width });
				randspot = (random_float() * 500.0) + idx;
			}

			path.render();
			Renderer::display();
			idx++;
		} 
#else
		bool abort;
		do {
			//std::cout << "Calculating..." << std::endl;
			output = path.evaluate(true);

			//std::cout << "Writing..." << std::endl;
			sock.write_to(&output.motion, sizeof(TankDriveCalculator::TankOutput));

			//std::cout << "Reading" << std::endl;
			sock.read_to(&abort, sizeof(bool)); //Read once before we update the spline
		} while (output.motion.special != TankDriveMotionUnit::Special::End && !abort);
#endif
	} 
}
