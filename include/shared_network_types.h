#ifndef SHARED_NETWORK_TYPES_H
#define SHARED_NETWORK_TYPES_H

struct TankDriveMotionUnit {
	float position_left;
	float velocity_left;
	float position_right;
	float velocity_right;
};

struct RioCommand {
	enum Type {
		RioNone,
		Stop,
		Motion,
		Request_Setup,
	} type;
	TankDriveMotionUnit motion;
	enum Action {
		None         = 0,
		Scissor_Up   = 1,
		Scissor_Down = 2,
		Cube_Expel   = 4,
		Cube_Intake  = 8,
	} action;
};

struct JetsonCommand {
	enum Type {
		JetsonNone,
		Setup,
		Request_Motion,
	} type;
	struct Setup {
		char field_colors[3];
		bool we_are_blue;
		int team_idx;
		float max_velocity_mps;
		float min_velocity_mps;
		float delta_time_ms;
		float wheel_width_mm;
		//TODO: Make the rio cache the last position in the even of a Jetson reboot you can recover the path
		//float estimated_position_x;
		//float estimated_position_y;
	} setup_data;
};

#endif
