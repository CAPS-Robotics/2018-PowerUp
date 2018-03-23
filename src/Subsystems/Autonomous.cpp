#include <Robot.h>
#include "Autonomous.h"

Autonomous::Autonomous() = default;

void Autonomous::Init(int station, std::string data) {
	//station = 6
	//roLeft = false;
	SmartDashboard::PutNumber("AutoPicked", station);
	state = 0;
	if(station == 1) {
		this->autoNum = 0;
	} else {
		this->autoNum = 1;
		this->roLeft = station == 0;
	}
	this->swLeft = data[0] == 'L';
	this->scLeft = data[1] == 'L';
	this->autoSc = swLeft == scLeft;
	Robot::vision->SetCamera(swLeft);
	timer = new Timer();
	Robot::arm->SetPosition(27);
}

void Autonomous::Loop() {
	SmartDashboard::PutNumber("Auton State", state);
	SmartDashboard::PutNumber("time", timer->Get());
	switch(autoNum) {
		/*case 0:
			StraightAhead(this->swLeft);
			SmartDashboard::PutString("sa", "a");
			break;*/
		case 0:
			HalfWay(this->swLeft);
			SmartDashboard::PutString("hw", "a");
			break;
		/*case 2:
			CrossField(this->swLeft);
			SmartDashboard::PutString("cf", "a");
			break;
		case 3:
			StraightScale(this->scLeft);
			SmartDashboard::PutString("ss", "a");
			break;
		case 4:
			CrossScale(this->scLeft);
			SmartDashboard::PutString("cs", "a");
			break;*/
		case 1:
			SideAuto(this->roLeft);
			SmartDashboard::PutString("gs", "a");
			break;
	}
}

void Autonomous::CrossField(bool left) {
	switch(state) {
		case -1:
			Robot::drivetrain->Brake();
			break;
		case 0:
			Robot::gyro->ResetHeading(0);
			timer->Reset();
			timer->Start();
			Robot::arm->MoveTo(20);
			state++;
			break;
		case 1:
			Robot::drivetrain->CrabDrive(left ? -1 : 1, .2, 0, .9, false);
			SmartDashboard::PutNumber("time", timer->Get());
			if(Robot::vision->GetCentralValue() > 120  + (left ? -20 : 20) && Robot::vision->GetCentralValue() < 200  + (left ? -20 : 20)) { state++; }
			if(timer->Get() > 1.5) { state = -1; }
			break;
		case 2:
			Robot::drivetrain->CrabDrive(0, 1, 0, .5, false);
			if(Robot::drivetrain->GetDistanceAway() < 42) state++;
			break;
		case 3:
			Robot::drivetrain->CrabDrive(0, 1, 0, .25, false);
			if(Robot::drivetrain->GetDistanceAway() < 12) {
				Robot::arm->Open();
				Robot::arm->KickDown();
				state++;
			}
			break;
		default:
			state = -1;
			this->GoAround(left);
			break;
	}
}

void Autonomous::StraightAhead(bool left) {
	switch(state) {
		case -1:
			Robot::drivetrain->Brake();
			break;
		case 0:
			Robot::gyro->ResetHeading(0);
			timer->Reset();
			timer->Start();
			Robot::arm->MoveTo(20);
			state++;
			break;
		case 1:
			Robot::drivetrain->CrabDrive(left ? .2 : -.2, 1, 0, .9, false);
			SmartDashboard::PutNumber("time", timer->Get());
			if(Robot::vision->GetCentralValue() > 120  + (left ? -20 : 20) && Robot::vision->GetCentralValue() < 200  + (left ? -20 : 20)) { state++; }
			if(timer->Get() > 1.5) { state = -1; }
			break;
		case 2:
			Robot::drivetrain->CrabDrive(0, 1, 0, .5, false);
			if(Robot::drivetrain->GetDistanceAway() < 42) state++;
			break;
		case 3:
			Robot::drivetrain->CrabDrive(0, 1, 0, .25, false);
			if(Robot::drivetrain->GetDistanceAway() < 12)  {
				Robot::arm->Open();
				Robot::arm->KickDown();
				state++;
			}
			break;
		default:
			state = -1;
			this->GoAround(left);
			break;
	}
}

void Autonomous::HalfWay(bool left) {
	switch(state) {
		case -1:
			Robot::drivetrain->Brake();
			break;
		case 0:
			Robot::gyro->ResetHeading(0);
			timer->Reset();
			timer->Start();
			Robot::arm->MoveTo(20);
			state++;
			break;
		case 1:
			Robot::drivetrain->CrabDrive(left ? -1 : 1, 1, 0, .9, false);
			if(Robot::vision->GetCentralValue() > 120 + (left ? -20 : 80) && Robot::vision->GetCentralValue() < 200 + (left ? -20 : 80)) {
				state++;
				timer->Reset();
				timer->Start();
			} else if(timer->Get() > 1.5) {
				state++;
				timer->Reset();
				timer->Start();
			}
			break;
		case 2:
			Robot::drivetrain->CrabDrive(0, 1, 0, .5, false);
			if(Robot::drivetrain->GetDistanceAway() < 36) {
				timer->Stop();
				state++;
			} else if(timer->Get() > 1.25)  {
				timer->Stop();
				state++;
			}
			break;
		case 3:
			Robot::drivetrain->CrabDrive(0, 1, 0, .25, false);
			if(Robot::drivetrain->GetDistanceAway() < 14 || timer->Get() > 1.25)  {
				Robot::arm->Open();
				Robot::arm->KickDown();
				timer->Reset();
				timer->Start();
				state++;
			}
			break;
		default:
			state = -1;
			this->GoAround(left);
			break;
	}
}

void Autonomous::GoAround(bool left) {
	switch(state) {
		case 4:
			Robot::drivetrain->CrabDrive(left ? -1 : 1, 0 , 0, .3, false);
			if(Robot::drivetrain->GetDistanceAway() > 50) {
				Robot::drivetrain->StartTravel();
				Robot::arm->MoveTo(0);
				state++;
			}
			break;
		case 5:
			Robot::drivetrain->CrabDrive(left ? -1 : 1, 0 , 0, .3, false);
			if(fabs(Robot::drivetrain->GetTravel()) >= 24 + left ? SONAR_CENTER : -SONAR_CENTER) {
				Robot::drivetrain->StartTravel();
				state++;
			}
			break;
		case 6:
			Robot::drivetrain->CrabDrive(0, 1, 0, .3, false);
			if(fabs(Robot::drivetrain->GetTravel()) >= 100) {
				Robot::drivetrain->StartTravel();
				state++;
			}
			break;
		case 7:
			Robot::drivetrain->CrabDrive(left ? -1 : 1, 0, 0, .5, false);
			if(fabs(Robot::drivetrain->GetTravel()) >= 30.5) state++;
			break;
		case 8:
			Robot::drivetrain->CrabDrive(0, 0, Robot::gyro->GetHeading() > 0 ? 1 : -1, 0.5, false);
			if(fabs(Robot::gyro->GetHeading() - 180) < 5) {
				Robot::arm->Open();
				Robot::arm->KickDown();
				state++;
			}
			break;
		case 9:
			Robot::drivetrain->CrabDrive(0, -1, 0, 0.25, false);
			if(Robot::drivetrain->GetDistanceAway() < 2) {
				Robot::arm->Close();
				state++;
			}
			break;
		default:
			state = -1;
			if(autoSc)
				ScaleAhead(scLeft);
			else
				ScaleAcross(scLeft);
			break;
		case -1:
			Robot::drivetrain->Brake();
			break;
	}
}

void Autonomous::ScaleAhead(bool left) {
	switch(state) {
		case 10:
			Robot::arm->MoveTo(72);
			state++;
			break;
		case 11:
			Robot::drivetrain->CrabDrive(0, 0, Robot::gyro->GetHeading() < 0 ? 1 : -1, 0.5, false);
			if(fabs(Robot::gyro->GetHeading()) < 5) {
				Robot::drivetrain->StartTravel();
				state++;
			}
			break;
		case 12:
			Robot::drivetrain->CrabDrive(left ? -1 : 1, 0, 0, .5, false);
			if(fabs(Robot::drivetrain->GetTravel()) >= 18) {
				Robot::drivetrain->StartTravel();
				Robot::arm->KickUp();
				state++;
			}
			break;
		case 13:
			Robot::drivetrain->CrabDrive(0, 1, 0, .75, false);
			if(fabs(Robot::drivetrain->GetTravel()) >= 65) {
				Robot::arm->Open();
				Robot::arm->KickDown();
				Robot::drivetrain->StartTravel();
				state++;
			}
			break;
		case 14:
			Robot::drivetrain->CrabDrive(0, -1, 0, .5, false);
			if(fabs(Robot::drivetrain->GetTravel()) >= 21) {
				Robot::arm->MoveTo(0);
				state++;
			}
			break;
		default:
			Robot::drivetrain->Brake();
			break;
	}
}

void Autonomous::ScaleAcross(bool left) {
	switch(state) {
		case 10:
			Robot::arm->MoveTo(72);
			state++;
			break;
		case 11:
			Robot::drivetrain->CrabDrive(0, 0, Robot::gyro->GetHeading() < 0 ? 1 : -1, 0.5, false);
			if(fabs(Robot::gyro->GetHeading()) < 5) {
				Robot::drivetrain->StartTravel();
				state++;
			}
			break;
		case 12:
			Robot::drivetrain->CrabDrive(left ? 1 : -1, 0, 0, .5, false);
			if(fabs(Robot::drivetrain->GetTravel()) >= 159) {
				Robot::drivetrain->StartTravel();
				Robot::arm->KickUp();
				state++;
			}
			break;
		case 13:
			Robot::drivetrain->CrabDrive(0, 1, 0, .75, false);
			if(fabs(Robot::drivetrain->GetTravel()) >= 65) {
				Robot::arm->Open();
				Robot::arm->KickDown();
				Robot::drivetrain->StartTravel();
				state++;
			}
			break;
		case 14:
			Robot::drivetrain->CrabDrive(0, -1, 0, .5, false);
			if(fabs(Robot::drivetrain->GetTravel()) >= 21) {
				Robot::arm->MoveTo(0);
				state++;
			}
			break;
		default:
			Robot::drivetrain->Brake();
			break;
	}
}

void Autonomous::StraightScale(bool left) {
	switch(state) {
		case 0:
			Robot::gyro->ResetHeading(left ? 90 : -90);
			//state++;
			break;
		case 1:
			Robot::drivetrain->CrabDrive(0, 1, 0, 0.7, false);
			if(fabs(Robot::drivetrain->GetDistanceAway()) < 24) {
				Robot::drivetrain->StartTravel();
				state++;
				SmartDashboard::PutNumber("AutoAdvance1", fabs(Robot::drivetrain->GetDistanceAway()));
			}
			break;
		case 2:
			Robot::drivetrain->CrabDrive(0, 1, 0, 0.7, false);
			if(fabs(Robot::drivetrain->GetTravel()) >= 48) {
				state++;
				Robot::arm->MoveTo(70);
				SmartDashboard::PutNumber("AutoAdvance2", fabs(Robot::drivetrain->GetTravel()));
			}
			break;
		case 3:
			Robot::drivetrain->CrabDrive(0, 1, 0, 0.7, false);
			if(fabs(Robot::drivetrain->GetTravel()) >= 108 + left ? SONAR_CENTER : -SONAR_CENTER) {
				state++;
				Robot::arm->Open();
				Robot::arm->KickDown();
				SmartDashboard::PutNumber("AutoAdvance3", fabs(Robot::drivetrain->GetTravel()));
				Robot::drivetrain->StartTravel();
			}
			break;
		case 4:
			Robot::drivetrain->CrabDrive(left ? -1 : 1, 0, 0, .5, false);
			if(fabs(Robot::drivetrain->GetTravel()) >= 21) {
				Robot::arm->MoveTo(0);
				state++;
				SmartDashboard::PutNumber("AutoAdvance4", fabs(Robot::drivetrain->GetTravel()));
			}
			break;
		default:
			Robot::drivetrain->Brake();
			break;
	}
}

void Autonomous::CrossScale(bool left) {
	switch(state) {
		case 0:
			Robot::gyro->ResetHeading(left ? 90 : -90);
			timer->Reset();
			timer->Start();
			state++;
			break;
		case 1:
			Robot::drivetrain->CrabDrive(0, 1, 0, 1, false);
			if(timer->Get() >= 1.25) {
				state++;
			}
			break;
		default:
			Robot::drivetrain->Brake();
	} //Bad
}

void Autonomous::SideAuto(bool left) {
	switch(state) {
		case 0:
			Robot::gyro->ResetHeading(left ? 90 : -90);
			Robot::drivetrain->StartTravel();
			state++;
			break;
		case 1:
			Robot::drivetrain->CrabDrive(0, 1, 0, 0.9, false);
			if(fabs(Robot::drivetrain->GetTravel()) >= ((scLeft == roLeft) ? 306 : 120)) {
				state++;
				if(scLeft == roLeft) { Robot::arm->MoveTo(70); }
			}
			break;
		case 2:
			Robot::drivetrain->CrabDrive(0, 0, Robot::gyro->GetHeading() > (left ? 90 : -90) ? 1 : -1, 0.4, false);
			if(fabs(Robot::gyro->GetHeading() + (left ? 90 : -90)) < 5) {
				state++;
				Robot::drivetrain->StartTravel();
			}
			break;
		case 3:
			Robot::drivetrain->Brake();
			if(Robot::arm->GetPosition() > 65 || scLeft != roLeft) {
				state++;
				timer->Reset();
				timer->Start();
			}
			break;
		case 4:
			if(scLeft != roLeft && swLeft != roLeft) {
				state = -1;
			}
			Robot::drivetrain->CrabDrive(roLeft ? 1 : -1, 0, 0, .3, false);
			if(fabs(Robot::drivetrain->GetTravel()) >= (scLeft == roLeft ? 6 : 18) || (timer->Get() > (scLeft == roLeft ? 0.5 : 2))) {
				state++;
				timer->Stop();
				Robot::arm->Open();
				Robot::arm->KickDown();
				Robot::drivetrain->StartTravel();
			}
			break;
		case 5:
			Robot::drivetrain->CrabDrive(roLeft ? -1 : 1, 0, 0, .5, false);
			if(fabs(Robot::drivetrain->GetTravel()) >= 18) {
				state++;
				Robot::arm->MoveTo(0);
			}
			break;
		default:
			Robot::drivetrain->Brake();
	}
}
