#include "OI.h"
#include "Robot.h"
#include "WPILib.h"


OI::OI() {
    for(bool &canPressi : canPress) {
        canPressi = true;
    }
    joy1 = new Joystick(0);
    buttonPad = new XboxController(1);
}

void OI::pollButtons() {
    //Block button
    //REMOVE BEFORE COMP
    //trolololol @Lucas
    /*if(joy1->GetRawButton(1)) {
        while(true) {}
    }*/
    if (joy1->GetRawButton(2)) {
        Robot::drivetrain->ReturnWheelsToZero();
    }
    if (joy1->GetRawButton(1)) {
        if (canPress[0]) {
            Robot::arm->ToggleIntake();
        }
        canPress[0] = false;
    } else { canPress[0] = true; }
    if(buttonPad->GetRawButton(11)) {
        if (canPress1[10]) {
            Robot::arm->ToggleKick();
        }
        canPress1[10] = false;
    } else { canPress1[10] = true; }
	if(buttonPad->GetRawButton(4)) {
		if (canPress1[3]) {
			Robot::drivetrain->Shift();
		}
		canPress1[3] = false;
	} else { canPress1[3] = true; }
    if(joy1->GetRawButton(6)) {
        Robot::gyro->ResetHeading(0);
    }
    if (this->GetStick() == 0) {
        if(fabs(Robot::arm->GetPosition() - Robot::arm->targetPos) < 1) Robot::arm->armMotor->Set(0);
    } else {
        Robot::arm->armMotor->Set(-this->GetStick());
        Robot::arm->targetPos = Robot::arm->GetPosition();
    }
    if (buttonPad->GetRawButton(1)) {
		if(canPress1[0]) {
			Robot::vision->SetCamera(1-Robot::vision->camera);
			canPress1[0] = false;
		}
	} else { canPress1[0] = true; }
    //Scale
    if (buttonPad->GetPOV(0) == 0) {
        Robot::arm->MoveTo(70);
    }
    //Intake
    if (buttonPad->GetPOV(0) == 180) {
        Robot::arm->MoveTo(0);
    }
    //Switch
    if (buttonPad->GetPOV(0) == 90) {
        Robot::arm->MoveTo(27);
    }
    //Reset
    if (buttonPad->GetPOV(0) == 270) {
        Robot::arm->SetPosition(0);
        Robot::arm->targetPos = 0;
    }
}

double OI::GetX() {
    return this->applyDeadzone(joy1->GetRawAxis(0), 0.15, 1);
}

double OI::GetY() {
    return this->applyDeadzone(-joy1->GetRawAxis(1), 0.15, 1);
}

double OI::GetTwist() {
    return this->applyDeadzone(joy1->GetRawAxis(2), 0.70, 1) / 2;
}

double OI::GetSlider() {
    return joy1->GetRawAxis(3);
}

double OI::GetAnalogY(int stickNum) {
    return this->applyDeadzone(buttonPad->GetRawAxis((stickNum*2)+1), 0.50, 1);
}

double OI::applyDeadzone(double val, double deadzone, double maxval) {
    if (fabs(val) <= deadzone) {
        return 0;
    }
    double sign = val / fabs(val);
    val = sign * maxval * (fabs(val) - deadzone) / (maxval - deadzone);
    return val;
}

double OI::GetStick() {
    return this->applyDeadzone(buttonPad->GetRawAxis(3), 0.50, 1);
}
