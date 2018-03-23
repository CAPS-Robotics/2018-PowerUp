#ifndef PigeonNav_H
#define PigeonNav_H

#include "ctre/Phoenix.h"
#include "WPILib.h"
#include "Commands/Subsystem.h"


class PigeonNav : public Subsystem, public PIDSource {
private:
    PigeonIMU * gyro;
    double * ypr;
	int offset;
public:
    PigeonNav();
    double GetHeading();
    double GetAngularRate();
    double PIDGet();
    void SetPIDSourceType(PIDSourceType pidSource);
    PIDSourceType GetPIDSourceType() const;
	void ResetHeading(int head);
};

#endif  // PigeonNav_H
