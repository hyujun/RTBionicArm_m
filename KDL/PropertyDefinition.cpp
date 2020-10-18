#include "PropertyDefinition.h"

#define ENC_OFFSET_J1 -223250
#define ENC_OFFSET_J2 8671266
#define ENC_OFFSET_J3 -15721256
#define ENC_OFFSET_J4 0
#define ENC_OFFSET_J5 0
#define ENC_OFFSET_J6 0

#define HARMONIC_100 100
#define GEAR_ELBOW 26*2
#define GEAR_PS 53
#define GEAR_WRIST 180

#define ABS_ENC_19 524288
#define ABS_ENC_18 262144
#define INC_ENC_10 1024*4

#define MAX_CURRENT_TYPE1 1.8
#define MAX_CURRENT_TYPE2 2
#define MAX_CURRENT_TYPE3 3.21
#define MAX_CURRENT_TYPE4 3.34
#define MAX_CURRENT_TYPE5 0.3

#define TORQUE_CONST_TYPE1 0.183 	// Nm/A Arm 1,2
#define TORQUE_CONST_TYPE2 0.091 	// Nm/A Arm3
#define TORQUE_CONST_TYPE3 0.0369 	// Nm/A elbow
#define TORQUE_CONST_TYPE4 0.014 	// Nm/A PS
#define TORQUE_CONST_TYPE5 0.0178 	// Nm/A wrist

// {w_x, w_y, w_z, q_x, q_y, q_z, l_x, l_y, l_z}
robot_kinematic_info serial_Kinematic_info[] = {
		{0.0, -1.0, 0.0,
		0.0, -0.1007, 0.0,
		0.0, -0.1547, -0.0},			// 1
		{1, 0, 0,
		0.0, -0.1547, 0.0,
		0.0, -0.1547, -0.1458},			// 2
		{0, 0, -1,
		0.0, -0.1547, -0.1458,
		0.0, -0.1547, -0.3602},			// 3
		{0, -1, 0,
		0.0, -0.1547, -0.3602,
		0.0, -0.1684, -0.3952},			// 4
		{0, 0, -1,
		0.0, -0.1684, -0.3952,
		0.0138, -0.1684, -0.6192},		// 5
		{0, 0, -1,
		0.0138, -0.1684, -0.6192,
		0.0138, -0.1684, -0.6792},		// 6
};

//{MASS,
// J_Ixx, J_Iyy, J_Izz, J_Ixy, J_Iyz, J_Izx,
// CoM_x, CoM_y, CoM_z},
robot_dynamic_info serial_Dynamic_info[] = {
		{0.9,
				0.43e-9, 0.34e-9, 0.96e-9, -0.54e-9, -0.98e-9, -0.35e-9,
				0.0, -0.1547, 0.0}, 	// 1
		{0.66,
				0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
				0.0, -0.1547, -0.11},			// 2
		{1.3,
				0.92e-9, 0.39e-9, 0.1e-9, 0.7e-9, -0.47e-9, -0.13e-9,
				0.0, -0.1547, -0.2558},			// 3
		{0.7,
				0.94e-9, 0.91e-9, 0.1e-9, 0.38e-9, -0.78e-9, -0.48e-9,
				0.0, -0.1547, -0.4302},			// 4
		{0.4,
				0.07e-9, 0.14e-9, 0.16e-9, 0.0, -0.95e-9, 0.0,
				0.0, -0.1684, -0.5152},			// 5
		{0.25,
				0.07e-9, 0.14e-9, 0.16e-9, 0.0, -0.95e-9, 0.0,
				0.0138, -0.1684, -0.6692},			// 6

};

// {HarmonicRatio, EncoderResolution, MaximumContinuousCurrent, TorqueConstant, AbsolutePositionOffset}
robot_motor_info serial_Motor_info[] = {
		{HARMONIC_100, ABS_ENC_18, MAX_CURRENT_TYPE1, TORQUE_CONST_TYPE1, ENC_OFFSET_J1},
		{HARMONIC_100, ABS_ENC_18, MAX_CURRENT_TYPE1, TORQUE_CONST_TYPE1, ENC_OFFSET_J2},
		{HARMONIC_100, ABS_ENC_18, MAX_CURRENT_TYPE2, TORQUE_CONST_TYPE2, ENC_OFFSET_J3},
		{GEAR_ELBOW, INC_ENC_10, MAX_CURRENT_TYPE3, TORQUE_CONST_TYPE3, ENC_OFFSET_J4},
		{GEAR_WRIST, INC_ENC_10, MAX_CURRENT_TYPE5, TORQUE_CONST_TYPE5, ENC_OFFSET_J6},
		{GEAR_PS, INC_ENC_10, MAX_CURRENT_TYPE4, TORQUE_CONST_TYPE4, ENC_OFFSET_J5},
};

FrictionMap frictionmap[] ={
		{0.05, 18.5},	//1
		{0.05, 10.5},	//2
		{0.05, 6.8},	//3
		{0.05, 7.0},	//4
		{0.05, 6.5},	//5
		{0.05, 4.0},	//6
};

FrictionTanh frictiontanh[] = {
		{215, 24.83, 22.54, 20.5, 11.81, 0.715}, 			// 1
		{215, 24.83, 22.54, 34.7, 11.81, 0.715},	 		// 2
		{41.87, 573.8, 592.2, 6.811, 12.96, 0.138}, 		// 3
		{19.99, 35.09, 64.18, 17.6, 36.34, 2.398e-7},		// 4
		{2.069, 59.79, 95.02, 18.5, 3.299, 2.49e-7},		// 5
		{151.3, 16.42, 17.26, 4.56, 14.47, 0.05998}, 		// 6
};



homing_info hominginfo[] = {
		//{-4369066, 	-2, ABS_ENC_18*4, 900}, //offset (int32_t)round((45.0/360.0)*ABS_ENC_18*HARMONIC_100)
		{-13107200,	-2, ABS_ENC_18*6, 900}, //offset (int32_t)round((180.0/360.0)*ABS_ENC_18*HARMONIC_100)
		{8956586, 	-1, ABS_ENC_18*6, 900}, //offset (int32_t)round((113.0/360.0)*ABS_ENC_18*HARMONIC_100)
		{-2184533, 	-2, ABS_ENC_18*6, 900}, //offset -(int32_t)round((30.0/360.0)*ABS_ENC_18*HARMONIC_100)
		{0, 		-1, INC_ENC_10*2, 900},
		{184320,	-1, INC_ENC_10*4, 700}, //offset (int32_t)round((90.0/360.0)*INC_ENC_10*GEAR_WRIST)
		{54272,		-1, INC_ENC_10*4, 300}, //offset (int32_t)round((90.0/360.0)*INC_ENC_10*GEAR_PS)
};
