#include <renderer.h>
#include <opencv2/opencv.hpp>
#include <socket.h>
#include <field_renderer.h>
#include <field_dimensions.h>
#include <spline_wrap.h>
#include <tank_drive_calculator.h>

#define JUST_RENDER true

//TODO: Add chaining for paths, or maybe a path stack
//      RX/TX structs layed out WELL (W/ actions or commands emittable by the jetson)
//      Add reversing mode for the tank drive calculator
namespace FD = FieldDimension;

int main () {
#if JUST_RENDER
	Renderer::init();
#else
	SocketServer sock(5070);
#endif

	while (true) {
		SplineWrap path (
				{ cv::Point2f(1000.0, 1000.0), 0.25, 1.0, MiscMath::pi / 2.0f, 660.0 / 2.0 },
				{ cv::Point2f(2000.0, 2000.0), 1.0, 0.25, MiscMath::pi / 2.0f, 660.0 / 2.0 }	
				);	

		TankDriveCalculator::TankOutput output;
#if JUST_RENDER
		do {
			Renderer::clear();
			FieldRenderer::render((char*)"LL", false);
			Renderer::bound(FD::field_bounds, 4.0);
			Renderer::grid(1000.0, 1000.0, 0.2, 0.2, 0.2, FD::field_bounds);
			output = TankDriveCalculator::evaluate(&path, 660.0 / 2.0, 20.0, false, true);
			Renderer::render_function_tank_drive(&path, 660.0 / 2.0, 20.0);
			Renderer::draw_robot(output.robot_direction, output.center_position, 700.0, 700.0, 0.8, 0.8, 0.8);
			Renderer::display();
		} while (output.motion.special != TankDriveMotionUnit::Special::End);
#else
		bool abort;
		do {
			std::cout << "Calculating..." << std::endl;
			output = path.evaluate(true);

			std::cout << "Writing..." << std::endl;
			sock.write_to(&output.motion, sizeof(TankDriveCalculator::TankOutput));

			std::cout << "Reading" << std::endl;
			sock.read_to(&abort, sizeof(bool)); //Read once before we update the spline
		} while (output.motion.special != TankDriveMotionUnit::Special::End && !abort);
#endif

	}
}
