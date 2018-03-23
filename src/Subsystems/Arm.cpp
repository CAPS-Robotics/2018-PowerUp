#include <Robot.h>
#include "Arm.h"

Arm::Arm() {
    this->armMotor = new WPI_TalonSRX(ARM_CIM);
    this->intake = new Intake();
    this->Close();
	this->KickUp();
    this->cimcoder = new Encoder(WINCH_CIMCODER_A, WINCH_CIMCODER_B);
    this->cimcoder->SetDistancePerPulse(WINCH_DIST_PER_PULSE);
    this->SetPosition(0);
    this->targetPos = cimcoder->GetDistance();
    //speed = new DoubleSolenoid(PCM, ARM_FAST, ARM_SLOW);
}

void Arm::Loop() {
    if(Robot::oi->GetStick() == 0) {
	    double aspeed;
	    aspeed = (this->targetPos - this->GetPosition())/10;
	    if(fabs((this->targetPos - this->GetPosition()) < 1)) aspeed = 0;
	    if(aspeed < -1) aspeed = -1;
	    if(aspeed > 1) aspeed = 1;
	    this->armMotor->Set(aspeed);
    }
}

void Arm::AutoLoop() {
	double aspeed;
	aspeed = (this->targetPos - this->GetPosition())/10;
	if(fabs((this->targetPos - this->GetPosition()) < 1)) aspeed = 0;
	if(aspeed < -1) aspeed = -1;
	if(aspeed > 1) aspeed = 1;
	this->armMotor->Set(aspeed);
}

void Arm::MoveTo(double position) {
    this->targetPos = position;
}

void Arm::ToggleIntake() {
    this->intakeClosed = this->intake->SetState(!intakeClosed);
}

void Arm::ToggleKick() {
    this->intakeKicked = this->intake->SetKicked(!intakeKicked);
}

void Arm::Close() {
    this->intakeClosed = this->intake->SetState(true);
}

void Arm::Open() {
    this->intakeClosed = this->intake->SetState(false);
}

void Arm::KickDown() {
    this->intakeKicked = this->intake->SetKicked(true);
}

void Arm::KickUp() {
    this->intakeKicked = this->intake->SetKicked(false);
}

//void Arm::ToggleShifter(){
  //  this->shifted=this->(!shifted);
    //if(shifted) {
      //  speed->Set(DoubleSolenoid::kReverse);
    //} else {
      //  speed->Set(DoubleSolenoid::kForward);
    //}
//}

double Arm::GetCurrent() {
    return this->armMotor->GetOutputCurrent();
}

double Arm::GetPosition() {
	return cimcoder->GetDistance() + offset;
}

void Arm::SetPosition(double position) {
	cimcoder->Reset();
	offset = position;
	MoveTo(position);
}
