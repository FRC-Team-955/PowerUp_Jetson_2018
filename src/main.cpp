#include <renderer.h>
#include <opencv2/opencv.hpp>
#include <socket.h>
#include <field_renderer.h>
#include <field_dimensions.h>
#include <A_to_B_calculator.h>

#define JUST_RENDER true

//TODO: Add chaining for paths
//      RX/TX structs layed out WELL (W/ actions or commands emittable by the jetson)
namespace FD = FieldDimension;

int main () {
	Renderer::init();

	//while (true) {
		AToB path (
				MiscMath::pi / 2.0, 					//direction start
				cv::Point2f(1000.0, 600.0/2.0),  //position start
				MiscMath::pi / 2.0,              //direction end
				FD::Switch::left_plate.tl() + cv::Point2f(FD::Switch::left_plate.width / 2.0, 0.0),  //position end
				660.0 / 2.0,                  	//wheel distance
				1.0,                    			//max allowed velocity
				20.0,                   			//max time step
				0.05);                  			//min velocity


	while (true) {
			TankDriveCalculator::TankOutput output = path.evaluate();
			Renderer::clear();
			FieldRenderer::render((char*)"LL", false);
			Renderer::bound(FD::field_bounds, 4.0);
			Renderer::grid(1000.0, 1000.0, 0.2, 0.2, 0.2, FD::field_bounds);
			Renderer::draw_robot(output.robot_direction, output.center_position, 700.0, 700.0, output.motion.velocity_left, 0.0, output.motion.velocity_right);
			Renderer::display();
			path.render();
			std::cout << "Finished cycle" << std::endl;
			if (output.motion.special == TankDriveMotionUnit::Special::End)
				break;
	}

	//float min = std::max(output.motion.velocity_left, output.motion.velocity_right);

	//}
}
