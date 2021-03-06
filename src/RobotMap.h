#ifndef FRC2018_ROBOTMAP_H
#define FRC2018_ROBOTMAP_H

//PID
const float SWERVE_MODULE_P = 5;
const float SWERVE_MODULE_I = 0;
const float SWERVE_MODULE_D = 3;
/*const float GYRO_P = 3;
const float GYRO_I = 0;
const float GYRO_D = 2;*/

//CAN IDs
const int FRONT_RIGHT_STEER = 1;
const int FRONT_RIGHT_DRIVE = 2;
const int FRONT_LEFT_DRIVE = 3;
const int FRONT_LEFT_STEER = 4;
const int BACK_LEFT_STEER = 5;
const int PIGEON_IMU_SRX = 5;
const int BACK_LEFT_DRIVE = 6;
const int ARM_CIM = 7;
const int BACK_RIGHT_DRIVE = 8;
const int BACK_RIGHT_STEER = 9;
const int PCM = 10;

//PCM Addresses
const int INTAKE_FORWARD = 5;
const int INTAKE_BACKWARD = 7;
const int INTAKE_KICK_FORWARD = 3;
const int INTAKE_KICK_BACKWARD = 0;
const int SHIFT_FORWARD = 1;
const int SHIFT_BACKWARD = 2;

//Inputs
const int FR_STEER_ENCODER = 0;
const int FL_STEER_ENCODER = 1;
const int BR_STEER_ENCODER = 2;
const int BL_STEER_ENCODER = 3;
const int RANGE_FINDER = 4;

const int WINCH_CIMCODER_A = 0;
const int WINCH_CIMCODER_B = 1;
const int DRIVE_CIMCODER_A = 2;
const int DRIVE_CIMCODER_B = 3;

//Offsets
const float FL_OFFSET = 3.455810193f;
const float FR_OFFSET = 2.0617673670000003f;
const float BR_OFFSET = 1.7419431810000001f;
const float BL_OFFSET = 3.2751461490000002f;
const float SONAR_CENTER = 0;

const int CAMERA_BRIGHTNESS = 0;
const int CAMERA_EXPOSURE = 8;
const double GR = 1.2;
const float GEAR_DISTANCE = 9.3;
const double PI = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679; //memz
const double WINCH_DIST_PER_PULSE = 3.0*PI/932.0;
const double DRIVE_DIST_PER_PULSE = 3.0*PI/100.0;


#endif //FRC2018_ROBOTMAP_H