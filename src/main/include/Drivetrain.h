#pragma once

#include <cmath>

#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/drive/DifferentialDrive.h>

#include <rev/CANSparkMax.h>

#include "Input.h"
#include "LimeLight.h"

class Drivetrain {
    private:
        double kP = 5e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000156, kMaxOutput = 1, kMinOutput = -1;
        double kMaxVel = 2000, kMinVel = 0, kMaxAcc = 1500, kAllErr = 0;

        rev::CANSparkMax driveMotor1{1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        rev::CANSparkMax driveMotor2{2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        rev::CANSparkMax driveMotor3{3, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        rev::CANSparkMax driveMotor4{4, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        rev::CANSparkMax driveMotor5{5, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        rev::CANSparkMax driveMotor6{6, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

        frc::MotorControllerGroup driveMotorGroupL{driveMotor1, driveMotor2, driveMotor3};
        frc::MotorControllerGroup driveMotorGroupR{driveMotor4, driveMotor5, driveMotor6};

    public:
        frc::DifferentialDrive differentialDrive{driveMotorGroupL, driveMotorGroupR};

        rev::SparkMaxRelativeEncoder leftDriveEncoder = driveMotor2.GetEncoder();
        rev::SparkMaxRelativeEncoder rightDriveEncoder = driveMotor5.GetEncoder();

        void Init(Drivetrain &driveTrain);
        void Teleop(Drivetrain &driveTrain, Input &input, LimeLight &limelight);
        bool InDeadzone(double input);
        void Disable();
};