#pragma once

#include <frc/DoubleSolenoid.h>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>

#include "Input.h"
#include "Constant.h"

class Intake {
    public:
        frc::DoubleSolenoid intakeSolenoid{20, Constant::pneumaticModule, 3, 2};

        ctre::phoenix::motorcontrol::can::WPI_VictorSPX intakeMotor{7};
        ctre::phoenix::motorcontrol::can::WPI_VictorSPX indexMotorR{8};
        ctre::phoenix::motorcontrol::can::WPI_VictorSPX indexMotorL{9};

        void Init(Intake &intake);
        void Teleop(Intake &intake, Input &input);
        void Autonomous();
        void Disable();
};