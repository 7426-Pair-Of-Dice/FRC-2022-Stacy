#pragma once

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableValue.h>

#include <cmath>

#include "../include/Constant.h"

class LimeLight {
    public:
        std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

        double targetFound = table->GetNumber("tv", 0.0);
        double targetOffsetHorizontal = table->GetNumber("tx", 0.0);
        double targetOffsetVertical = table->GetNumber("ty", 0.0);
        double targetArea = table->GetNumber("ta", 0.0);
        double targetSkew = table->GetNumber("ts", 0.0);

        void UpdateLimelightValues();
        void EnableLEDs();
        void DisableLEDs();
        double GetDistanceFromTarget();
        double GetFlywheelSpeedFromDistance(double distance);
        
};
