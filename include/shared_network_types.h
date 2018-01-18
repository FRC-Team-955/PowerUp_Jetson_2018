#ifndef SHARED_NETWORK_TYPES_H
#define SHARED_NETWORK_TYPES_H

struct TankDriveMotionUnit {
	float position_left;
	float velocity_left;
	float position_right;
	float velocity_right;
	float delta_time;
	enum Special {
		Beginning = 0,
		Middle = 1,
		End = 2
	} special;
};

#endif
