#pragma once

#include <ctre/Phoenix.h>

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Input.h"
#include "LimeLight.h"

class Shooter {
    public:
        TalonFX shooterMotorL{10};
        TalonFX shooterMotorR{11};

        void Init(Shooter &shooter);
        void Teleop(Shooter &shooter, Input &input, LimeLight &limelight);
        void Disable();
};