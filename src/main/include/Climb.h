#pragma once

#include <frc/DoubleSolenoid.h>

#include <ctre/Phoenix.h>

#include "Input.h"
#include "Constant.h"

class Climb {
    private:
        frc::DoubleSolenoid climbArmSolenoid{20, Constant::pneumaticModule, 4, 5};
        frc::DoubleSolenoid climbHookSolenoid{20, Constant::pneumaticModule, 0, 1};

        TalonFX climbMotor{12};

        double lastPosition = climbMotor.GetSelectedSensorPosition(0);
    public:
        void Init(Climb &climb);
        void Teleop(Climb &climb, Input &input);
        void Autonomous();
};