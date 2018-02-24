#ifndef FRC2018_ARM_H
#define FRC2018_ARM_H

#include "wpilib.h"
#include "ctre/Phoenix.h"
#include "RobotMap.h"
#include "Intake.h"
#include <string>

class Arm {
private:
public:
    Arm();
    WPI_TalonSRX * armMotor;
    Intake * intake;
    Encoder * cimcoder;
    double position;
    bool intakeClosed;
    bool intakeKicked;
    double targetPos;
    void Loop();
    void ToggleIntake();
    void ToggleKick();
    void Close();
    void Open();
    void MoveTo(double position);
    double GetCurrent();
	void KickUp();
	void KickDown();
};


#endif //FRC2018_ARM_H
