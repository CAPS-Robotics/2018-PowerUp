#ifndef FRC2018_AUTONOMOUS_H
#define FRC2018_AUTONOMOUS_H

#include "string"

class Autonomous {
private:
	int autoNum;
	bool swLeft;
	bool scLeft;
	bool roLeft;
	bool autoSc;
	int state;
	Timer * timer;
public:
	Autonomous();
	void Init(int station, std::string data);
	void Loop();
	void CrossField(bool left);
	void StraightAhead(bool left);
	void HalfWay(bool left);
	void GoAround(bool left);
	void ScaleAhead(bool left);
	void ScaleAcross(bool left);

	void StraightScale(bool left);

	void CrossScale(bool left);

	void SideAuto(bool left);

	void ScaleAuto(bool left);
};

#endif //FRC2018_AUTONOMOUS_H