#ifndef FRC2018_VISION_H
#define FRC2018_VISION_H

#include "WPILib.h"
#include "Commands/Subsystem.h"
#include <vector>

class Vision {
private:
	cs::VideoSink server;
	std::shared_ptr<NetworkTable> table;
	std::vector<double> centerX;
	std::vector<double> centerY;
	std::vector<double> height;
	std::vector<double> width;
	cs::CvSink * cvsink0;
	cs::CvSink * cvsink1;
	/*cs::CvSource sdb0;
	cs::CvSource sdb1;*/
public:
	cs::UsbCamera * cam0;
	cs::UsbCamera * cam1;
	int camera;
	Vision();
	void Update();
	double GetCentralValue();
	void SetCamera(int camera);
	//void VisionThread();
};


#endif