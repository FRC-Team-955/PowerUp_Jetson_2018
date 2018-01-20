#include <renderer.h>
#include <opencv2/opencv.hpp>
#include <socket.h>
#include <field_renderer.h>
#include <field_dimensions.h>
#include <A_to_B_calculator.h>
#include <sine_wave_calculator.h>

#define JUST_RENDER false

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
		AToB path (
				MiscMath::pi / 2.0, 					//direction start
				cv::Point2f(1000.0, 1000.0), //position start
				MiscMath::pi / 2.0,              //direction end
				cv::Point2f(2000.0, 3000.0),
				//FD::Switch::left_plate.tl() + cv::Point2f(FD::Switch::left_plate.width / 2.0, 0.0),  //position end
				660.0 / 2.0,                  	//wheel distance
				1.0,                    			//max allowed velocity
				10.0,                   			//max time step
				0.1);                  			//min velocity
		/*
			SineWaveCalculator path (
			cv::Point2f(1000.0, 3000.0), //Start from
			1000.0,			//Amplitude
			660.0 / 2.0,			//Period
			20.0,				//max time step
			660.0 / 2.0,   //wheel distance
			4);            //Repeat 
			*/

		TankDriveCalculator::TankOutput output;
#if JUST_RENDER
		do {
			Renderer::clear();
			FieldRenderer::render((char*)"LL", false);
			Renderer::bound(FD::field_bounds, 4.0);
			Renderer::grid(1000.0, 1000.0, 0.2, 0.2, 0.2, FD::field_bounds);
			output = path.evaluate(true);
			Renderer::draw_robot(output.robot_direction, output.center_position, 700.0, 700.0, 0.8, 0.8, 0.8);
			path.render();
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
