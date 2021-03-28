#pragma once
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

class LimeLight
{

public:
    std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

    double tx = table->GetNumber("tx", 0.0);
    double ty = table->GetNumber("ty", 0.0);
    double ta = table->GetNumber("ta", 0.0);
    double tv = table->GetNumber("tv", 0.0);

    double targetPositionY = 0;
    double limelightHasTarget = table->GetNumber("tv", 0.0);
    int ledMode = table->PutNumber("ledMode", ledMode);

};
