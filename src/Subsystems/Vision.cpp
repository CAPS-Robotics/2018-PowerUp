#include "Vision.h"
#include "../RobotMap.h"
#include <vector>
#include <networktables/NetworkTableInstance.h>
#include <Robot.h>
#include <opencv2/core/mat.hpp>
#include <string>

Vision::Vision() {
	cam0 = new cs::UsbCamera("cam0", 0);
	cam0->SetResolution(320, 240);
	cam0->SetBrightness(CAMERA_BRIGHTNESS);
	cam0->SetFPS(10);
	cam0->SetExposureManual(CAMERA_EXPOSURE);
	cam1 = new cs::UsbCamera("cam1", 1);
	cam1->SetResolution(320, 240);
	cam1->SetBrightness(CAMERA_BRIGHTNESS);
	cam1->SetFPS(10);
	cam1->SetExposureManual(CAMERA_EXPOSURE);
	CameraServer::GetInstance()->PutVideo("GRIPCam", 320, 240);
	/*sdb0 = CameraServer::GetInstance()->PutVideo("cam0", 320, 240);
	sdb1 = CameraServer::GetInstance()->PutVideo("cam1", 320, 240);*/
	cvsink0 = new cs::CvSink("cam0cv");
	cvsink0->SetSource(*cam0);
	cvsink0->SetEnabled(true);
	cvsink1 = new cs::CvSink("cam1cv");
	cvsink1->SetSource(*cam1);
	cvsink1->SetEnabled(true);
	CameraServer::GetInstance()->StartAutomaticCapture(*cam0);
	CameraServer::GetInstance()->StartAutomaticCapture(*cam1);
	server = CameraServer::GetInstance()->GetServer();
	SetCamera(0);
	nt::NetworkTableInstance::GetDefault().GetEntry("/CameraPublisher/GRIPCam/streams").SetStringArray(*new std::string[2]{"mjpeg:http://roborio-2410-frc.local:1181/?action=stream, mjpeg:http://10.24.10.2:1181/?action=stream"});
	table = nt::NetworkTableInstance::GetDefault().GetTable("GRIP/AllDemContours");
	this->Update();
}

/*void Vision::VisionThread() {
	cv::Mat img0;
	cv::Mat img1;
	cvsink0->GrabFrame(img0);
	cvsink1->GrabFrame(img1);
	sdb0.PutFrame(img0);
	sdb1.PutFrame(img1);
}*/

void Vision::Update() {
	centerX = table->GetNumberArray("centerX", llvm::ArrayRef<double>());
	centerY = table->GetNumberArray("centerY", llvm::ArrayRef<double>());
	height = table->GetNumberArray("height", llvm::ArrayRef<double>());
	width = table->GetNumberArray("width", llvm::ArrayRef<double>());
}

double Vision::GetCentralValue() {
	this->Update();
	double theCenterX = 0;
	for(unsigned int i = 0; i < centerX.size(); i++) {
		theCenterX += centerX[i];
	}
	if(centerX.size() != 0) { theCenterX /= centerX.size(); }
	return theCenterX;
}

void Vision::SetCamera(int camera) {
	this->camera = camera;
	if(camera == 0) {
		server.SetSource(*cam0);
	} else {
		server.SetSource(*cam1);
	}
}
