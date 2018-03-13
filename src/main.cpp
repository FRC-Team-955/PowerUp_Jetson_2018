#include <field_dimensions.h>
#include <field_renderer.h>
#include <multi_waypoint_calculator.h>
#include <opencv2/opencv.hpp>
#include <renderer.h>
#include <socket.h>

#define RENDER true

namespace FD = FieldDimension;
namespace MM = MiscMath;
#define JSL JetsonCommand::Setup::LayoutBits

int main() {
#if RENDER
	Renderer::init();
#endif
	std::cout << "Waiting for connection..." << std::endl;
	SocketServer rio(5801);

	RioCommand output_command;
	JetsonCommand input_command;
	JetsonCommand setup_info;

	//while (rio.is_open()) {
	while (true) {
		//Wait for setup data
		setup_info.type = JetsonCommand::Type::JetsonNone;
		while (setup_info.type != JetsonCommand::Type::Setup) {
			//TODO: Recoverable reboots, queries where the bot thinks it is and sets self to that position
			output_command.type = RioCommand::Type::Request_Setup;
			rio.write_to(&output_command, sizeof(RioCommand));
			rio.read_to(&setup_info, sizeof(JetsonCommand)); 
		}

		// Create a path based on the field colors and our position
		MultiWaypointCalculator path(setup_info.setup_data.wheel_width/ 2.0, setup_info.setup_data.delta_time);

		if (setup_info.setup_data.layout_bits & JSL::we_are_left) {
			path.reset_and_begin(
					{cv::Point2f(1000.0, 1000.0),
					setup_info.setup_data.min_velocity, setup_info.setup_data.max_velocity, MM::pi / 2.0f, setup_info.setup_data.wheel_width});
		} else {
			path.reset_and_begin(
					{cv::Point2f(5000.0, 1000.0),
					setup_info.setup_data.min_velocity, setup_info.setup_data.max_velocity, MM::pi / 2.0f, setup_info.setup_data.wheel_width});
		}

		bool left_both = setup_info.setup_data.layout_bits & JSL::switch_left && setup_info.setup_data.layout_bits & JSL::scale_left;
		bool right_both = !(setup_info.setup_data.layout_bits & JSL::switch_left || setup_info.setup_data.layout_bits & JSL::scale_left);
		if (left_both) {
			path.push_back(
					{FD::Switch::front_left + cv::Point2f(0.0, -setup_info.setup_data.wheel_width / 2.0),
					setup_info.setup_data.min_velocity, setup_info.setup_data.max_velocity, MM::pi / 2.0f, setup_info.setup_data.wheel_width}, false, RioCommand::Action::Cube_Intake);
		} else if (right_both) {
			path.push_back(
					{FD::Switch::front_right + cv::Point2f(0.0, -setup_info.setup_data.wheel_width / 2.0),
					setup_info.setup_data.min_velocity, setup_info.setup_data.max_velocity, MM::pi / 2.0f, setup_info.setup_data.wheel_width}, false, RioCommand::Action::Cube_Intake);
		}

		if (setup_info.setup_data.layout_bits & JSL::scale_left) {
			path.push_back(
					{FD::Switch::front_left + cv::Point2f(-1500.0, -1000.0),
					left_both ? setup_info.setup_data.min_velocity : setup_info.setup_data.max_velocity, setup_info.setup_data.max_velocity, MM::pi / 2.0f, setup_info.setup_data.wheel_width}, left_both, RioCommand::Action::Cube_Intake);
			path.push_back(
					{FD::Scale::front_left + cv::Point2f(0.0, -setup_info.setup_data.wheel_width / 2.0),
					setup_info.setup_data.min_velocity, setup_info.setup_data.max_velocity, MM::pi / 2.0f, setup_info.setup_data.wheel_width}, false, RioCommand::Action::Cube_Intake);
		} else {
			path.push_back(
					{FD::Switch::front_right + cv::Point2f(1500.0, -1000.0),
					right_both ? setup_info.setup_data.min_velocity : setup_info.setup_data.max_velocity, setup_info.setup_data.max_velocity, MM::pi / 2.0f, setup_info.setup_data.wheel_width}, right_both, RioCommand::Action::Cube_Intake);
			path.push_back(
					{FD::Scale::front_right + cv::Point2f(0.0, -setup_info.setup_data.wheel_width / 2.0),
					setup_info.setup_data.min_velocity, setup_info.setup_data.max_velocity, MM::pi / 2.0f, setup_info.setup_data.wheel_width}, false, RioCommand::Action::Cube_Intake);
		}

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
			FieldRenderer::render(setup_info.setup_data.layout_bits);
			path.render();
			Renderer::display();
#endif
		}
		std::cout << "Ran out; Sent stop." << std::endl;
		output_command.type = RioCommand::Type::Stop;
		rio.write_to(&output_command, sizeof(output_command));
	}
}
