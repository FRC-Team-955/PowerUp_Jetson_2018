#include <renderer.h>
#include <goal_path_calculator.h>
#include <path_calculator.h>
#include <opencv2/opencv.hpp>
#include <socket.h>
#include <multiple_waypoint_path_creator.h>

#define JUST_RENDER true
void test_graphical();

bool serve_path (Path path, SocketServer sock) {
	Path::TalonPoint next;
	do {
		std::cout << "Writing" << std::endl;
		sock.write_to(&next, sizeof(Path::TalonPoint));
		bool abort;
		std::cout << "Reading" << std::endl;
		sock.read_to(&abort, sizeof(bool)); //Read once before we update the spline
		return true;
	} while(path.next_point(&next));
	return false;
}

void walk_path_render (Path path) {
	Path::TalonPoint next;
	while(path.next_point(&next)) {
		Renderer::clear();
		auto view = cv::Rect2f(-3000, -3000, 6000, 6000);
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

int main () {
	const double pi = std::acos(-1);

#if JUST_RENDER
	Renderer::init();
#else
	SocketServer sock(5070);
#endif

	MultipleWaypointPathCreator creator (2, 10.0, 660.0 / 2.0);	
	while (true) {
		for (float i = 0; i < (pi * 2.0); i+= pi/7.0) {
			Path path = GoalPathCalculator::calculate_path(
					pi / 2.0, 							//direction start
					cv::Point2f(0.0, 0.0),  		//position start
					-pi / 2.0,               					//direction end
					MiscMath::RadialOffset(i, 2000.0, cv::Point2f(0.0, 0.0)), //position end
					660.0 / 2.0,                  //wheel distance
					1.0,                    		//max allowed velocity
					20.0,                   		//max time step
					0.05);                  		//min velocity
#if JUST_RENDER
			walk_path_render(path);
#else
			serve_path(path, sock);
#endif
		}
	}
}
