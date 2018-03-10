#include <field_dimensions.h>
#include <field_renderer.h>
#include <multi_waypoint_calculator.h>
#include <opencv2/opencv.hpp>
#include <renderer.h>
#include <socket.h>

#define RENDER true

namespace FD = FieldDimension;
namespace MM = MiscMath;

int main() {
#if RENDER
	Renderer::init();
#endif
	std::cout << "Waiting for connection..." << std::endl;
	SocketServer rio(5801);

	const float wheel_width = 660.0;

	RioCommand output_command;
	JetsonCommand input_command;
	JetsonCommand setup_info;

	//while (rio.is_open()) {
	while (true) {
		//Wait for setup data
		while (setup_info.type != JetsonCommand::Type::Setup) {
			//TODO: Recoverable reboots, queries where the bot thinks it is and sets self to that position
			output_command.type = RioCommand::Type::Request_Setup;
			rio.write_to(&output_command, sizeof(RioCommand));
			rio.read_to(&setup_info, sizeof(JetsonCommand)); 
		}
		// Create a path based on the field colors and our position
		MultiWaypointCalculator path(setup_info.setup_data.wheel_width_mm / 2.0, setup_info.setup_data.delta_time_ms);

		//Start
		path.reset_and_begin(
				{cv::Point2f(1000.0, 1000.0),
				setup_info.setup_data.min_velocity_mps, setup_info.setup_data.max_velocity_mps, MM::pi / 2.0f, wheel_width});

		//Switch front
		path.push_back(
				{FD::Switch::front,
				setup_info.setup_data.min_velocity_mps, setup_info.setup_data.max_velocity_mps, MM::pi / 2.0f, wheel_width}, false, RioCommand::Action::Cube_Intake);

		TankDriveCalculator::TankOutput tank_output;
		while (path.evaluate(tank_output, output_command.action)) {
			output_command.type = RioCommand::Type::Motion;
			output_command.motion = tank_output.motion;
			rio.write_to(&output_command, sizeof(output_command));
			rio.read_to(&input_command, sizeof(input_command)); 
			if (input_command.type == JetsonCommand::Type::Setup) {
				break;
			}
#if RENDER
			Renderer::clear();
			Renderer::bound(FD::field_bounds, 4.0);
			Renderer::grid(1000.0, 1000.0, 0.2, 0.2, 0.2, FD::field_bounds);
			FieldRenderer::render(setup_info.setup_data.field_colors, setup_info.setup_data.we_are_blue);
			path.render();
			Renderer::display();
#endif
		}
		std::cout << "Ran out; Sent stop." << std::endl;
		output_command.type = RioCommand::Type::Stop;
		rio.write_to(&output_command, sizeof(output_command));
	}
}
