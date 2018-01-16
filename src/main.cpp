#include <renderer.h>
#include <goal_path_calculator.h>
#include <path_calculator.h>
#include <opencv2/opencv.hpp>
#include <socket.h>
#include <multiple_waypoint_path_creator.h>
#include <field_renderer.h>
#include <field_dimensions.h>

#define JUST_RENDER true

//TODO: Add chaining for paths
//      Make a tank drive class to replace paths, generic over all sq differentiable functions
//      RX/TX structs layed out WELL (W/ actions or commands emittable by the jetson)
//      Finish the field rendering class (make it only a namespace?)
namespace FD = FieldDimension;
const double pi = std::acos(-1);

void draw_robot_follow_path(Path path, FieldRenderer f_renderer);
bool socket_serve(Path path, SocketServer sock);

int main () {

#if JUST_RENDER
	FieldRenderer f_renderer ((char*)"TODO");
	Renderer::init();
#else
	SocketServer sock(5070);
#endif

	MultipleWaypointPathCreator creator (2, 10.0, 660.0 / 2.0);	
	while (true) {
		{
			Path path = GoalPathCalculator::calculate_path(
					pi / 2.0, 							//direction start
					cv::Point2f(1000.0, 600.0/2.0),  	//position start
					pi / 2.0,               		//direction end
					cv::Point2f(
						FD::switch_plate_horizontal_offset + (FD::switch_plate_width / 2.0), 
						FD::switch_plate_vertical_offset
						), 								//position end
					660.0 / 2.0,                  //wheel distance
					1.0,                    		//max allowed velocity
					20.0,                   		//max time step
					0.05);                  		//min velocity

#if JUST_RENDER
			draw_robot_follow_path(path, f_renderer);
#else
			socket_serve(path, sock);
#endif
		}
		{
			Path path = GoalPathCalculator::calculate_path(
					pi / 2.0,               		//direction end
					cv::Point2f(
						FD::switch_plate_horizontal_offset + (FD::switch_plate_width / 2.0), 
						FD::switch_plate_vertical_offset
						), 								//position end
					pi / 2.0, 							//direction start
					cv::Point2f(4000.0, 600.0/2.0),  	//position start
					660.0 / 2.0,                  //wheel distance
					1.0,                    		//max allowed velocity
					20.0,                   		//max time step
					0.05);                  		//min velocity

#if JUST_RENDER
			draw_robot_follow_path(path, f_renderer);
#else
			socket_serve(path, sock);
#endif
		}
	}
}

void draw_robot_follow_path(Path path, FieldRenderer f_renderer) {
	Path::TalonPoint next;
	while(path.next_point(&next)) {
		Renderer::clear();
		auto view = f_renderer.render();
		Renderer::bound(view, 4.0);
		Renderer::grid(1000.0, 1000.0, 0.2, 0.2, 0.2, view);

		path.render();
		Renderer::render_spline(&path.spline);

		float robot_angle = 0.0;
		cv::Point2f robot_pos (0.0, 0.0);
		path.get_current_loc_nondestruct(&robot_pos, &robot_angle);
		float min = std::max(next.velocity_left, next.velocity_right);
		Renderer::draw_robot(robot_angle, robot_pos, 700.0, 700.0, -min, min, 0.0);

		Renderer::display();
	}
}


bool socket_serve(Path path, SocketServer sock) {
	Path::TalonPoint next;
	do {
		std::cout << "Writing" << std::endl;
		sock.write_to(&next, sizeof(Path::TalonPoint));
		bool abort;
		std::cout << "Reading" << std::endl;
		sock.read_to(&abort, sizeof(bool)); //Read once before we update the spline
		return true;
	} while(path.next_point(&next));
}
