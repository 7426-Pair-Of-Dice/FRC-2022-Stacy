#include "../include/Limelight.h"

void LimeLight::UpdateLimelightValues() {
    targetFound = this->table->GetNumber("tv", 0.0);
    targetOffsetHorizontal = this->table->GetNumber("tx", 0.0);
    targetOffsetVertical = this->table->GetNumber("ty", 0.0);
    targetArea = this->table->GetNumber("ta", 0.0);
    targetSkew = this->table->GetNumber("ts", 0.0);
}

void LimeLight::EnableLEDs() {
    table->PutNumber("ledMode", 3);
}

void LimeLight::DisableLEDs() {
    table->PutNumber("ledMode", 1);
}

double LimeLight::GetDistanceFromTarget() {
    if (targetFound == 1) {
        double a1 = targetOffsetVertical;
        double a2 = Constant::limelightAngle;

        double goalangle_degrees = a1 + a2;
        double goalangle_radians = (goalangle_degrees * 3.14) / 180;

        double height = Constant::goalHeight - Constant::limelightHeight;

        return (height / std::tan(goalangle_radians));
    } else {
        return 0;
    }
}

double LimeLight::GetFlywheelSpeedFromDistance(double distance) {
    if (this->targetFound == 1) {
        return 0.0000812102 * (distance * distance * distance) + 256.338;
    } else {
        return 370;
    }
}
