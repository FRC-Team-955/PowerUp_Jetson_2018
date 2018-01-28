#include <renderer.h>
#include <opencv2/opencv.hpp>
#include <socket.h>
#include <field_renderer.h>
#include <field_dimensions.h>
#include <tank_drive_calculator.h>

#define JUST_RENDER true

//TODO:
//Reverse path rendering
//Path chaining w/ hot-swappable position system
//Find some way to manage some splines starting at 1.0 and ending at 0.0
namespace FD = FieldDimension;

int main () {
#if JUST_RENDER
	Renderer::init();
#else
	SocketServer sock(5070);
#endif

	while (true) {
		bool rev = true; 
		TankDriveCalculator calc (SplineWrap (
					{ cv::Point2f(1000.0, 3000.0), 0.25, 1.0, MiscMath::pi / 2.0f, 660.0 * 2.0, rev },
					{ cv::Point2f(2000.0, 1000.0), 0.25, 1.0, MiscMath::pi / 2.0f, 660.0 * 2.0, rev }
					), 660.0 / 2.0, 20.0, rev);	

		TankDriveCalculator::TankOutput output;
#if JUST_RENDER
		do {
			Renderer::clear();
			FieldRenderer::render((char*)"LL", false);
			Renderer::bound(FD::field_bounds, 4.0);
			Renderer::grid(1000.0, 1000.0, 0.2, 0.2, 0.2, FD::field_bounds);

			output = calc.evaluate(true);

			calc.render();
			calc.render_robot();

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
