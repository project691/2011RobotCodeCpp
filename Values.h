//Values.h

#ifndef VALUES_H
#define VALUES_H

	const int RIGHT_JOYSTICK = 1;
	const int LEFT_JOYSTICK = 2;
	const int LIFT_JOYSTICK = 3;

	const int FR_DRIVE_TALON		= 1;
	const int FL_DRIVE_TALON		= 10;
	const int BR_DRIVE_TALON		= 2;
	const int BL_DRIVE_TALON		= 8;

	const int FR_DRIVE_ENCODER_A	= 1;
	const int FR_DRIVE_ENCODER_B	= 2;
	const double FR_DRIVE_ENCODER_DISTANCE_PER_PULSE = 1.0;
	const bool FR_DRIVE_ENCODER_REVERSE = false;

	const int FL_DRIVE_ENCODER_A	= 3;
	const int FL_DRIVE_ENCODER_B	= 4;
	const double FL_DRIVE_ENCODER_DISTANCE_PER_PULSE = 1.0;
	const bool FL_DRIVE_ENCODER_REVERSE = false;

	const int BR_DRIVE_ENCODER_A	= 5;
	const int BR_DRIVE_ENCODER_B	= 6;
	const double BR_DRIVE_ENCODER_DISTANCE_PER_PULSE = 1.0;
	const bool BR_DRIVE_ENCODER_REVERSE = false;

	const int BL_DRIVE_ENCODER_A	= 7;
	const int BL_DRIVE_ENCODER_B	= 8;
	const double BL_DRIVE_ENCODER_DISTANCE_PER_PULSE = 1.0;
	const bool BL_DRIVE_ENCODER_REVERSE = false;

	const double DRIVE_PID_SCALAR = 650.0;	//TODO: This is why PID is slow!
	const double DRIVE_PID_KP = 0.4;
	const double DRIVE_PID_KI = 0.0;
	const double DRIVE_PID_KD = 0.0005;
	const double DRIVE_PID_KF = 0.5;
	const double DRIVE_PID[5] = {DRIVE_PID_KP, DRIVE_PID_KI, DRIVE_PID_KD, DRIVE_PID_KF, DRIVE_PID_SCALAR};

	const int LIFT_TALON = 9;

	const int CLAW_RELAY = 1;

	const int MINIBOT_RELAY = 8;

#endif //VALUES_H
