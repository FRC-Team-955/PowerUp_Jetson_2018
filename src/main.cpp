#include <renderer.h>
#include <opencv2/opencv.hpp>
#include <socket.h>
#include <field_renderer.h>
#include <field_dimensions.h>
#include <A_to_B_calculator.h>

#define JUST_RENDER true

//TODO: Add chaining for paths
//      Make a tank drive class to replace paths, generic over all sq differentiable functions
//      RX/TX structs layed out WELL (W/ actions or commands emittable by the jetson)
//      Finish the field rendering class (make it only a namespace?)
namespace FD = FieldDimension;

void draw_robot_follow_path(AToB path);
//bool socket_serve(AToB path, SocketServer sock);

int main () {

#if JUST_RENDER
	Renderer::init();
#else
	SocketServer sock(5070);
#endif

	while (true) {
		{
			AToB path (
					MiscMath::pi / 2.0, 							//direction start
					cv::Point2f(1000.0, 600.0/2.0),  	//position start
					MiscMath::pi / 2.0,               		//direction end
					FD::Switch::left_plate.tl() + cv::Point2f(FD::Switch::left_plate.width / 2.0, 0.0), 
					660.0 / 2.0,                  //wheel distance
					1.0,                    		//max allowed velocity
					20.0,                   		//max time step
					0.05);                  		//min velocity

#if JUST_RENDER
			draw_robot_follow_path(path);
#else
			//socket_serve(path, sock);
#endif
		}
	}
}

void draw_robot_follow_path(AToB path) {
	while(true) {
		TankDriveCalculator::TankOutput output = path.evaluate(true);
		Renderer::clear();
		FieldRenderer::render((char*)"LL", false);
		Renderer::bound(FD::field_bounds, 4.0);
		Renderer::grid(1000.0, 1000.0, 0.2, 0.2, 0.2, FD::field_bounds);
		path.render();

		float min = std::max(output.motion.velocity_left, output.motion.velocity_right);
		Renderer::draw_robot(output.robot_direction, output.center_position, 700.0, 700.0, -min, min, 0.0);

		Renderer::display();
	}
}


/*
bool socket_serve(AToB path, SocketServer sock) {
	TankDriveMotionUnit next;
	do {
		std::cout << "Writing" << std::endl;
		sock.write_to(&next, sizeof(TankDriveMotionUnit));
		bool abort;
		std::cout << "Reading" << std::endl;
		sock.read_to(&abort, sizeof(bool)); //Read once before we update the spline
		return true;
	} while(path.next_point(&next));
}
*/
