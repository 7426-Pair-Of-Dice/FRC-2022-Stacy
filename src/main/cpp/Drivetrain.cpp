#include "../include/Drivetrain.h"
#include "../include/Constant.h"

void Drivetrain::Init(Drivetrain &driveTrain) {
    driveTrain.driveMotorGroupL.SetInverted(true);

    driveTrain.driveMotor1.SetOpenLoopRampRate(Constant::openLoopRampRate);
    driveTrain.driveMotor2.SetOpenLoopRampRate(Constant::openLoopRampRate);
    driveTrain.driveMotor3.SetOpenLoopRampRate(Constant::openLoopRampRate);
    driveTrain.driveMotor4.SetOpenLoopRampRate(Constant::openLoopRampRate);
    driveTrain.driveMotor5.SetOpenLoopRampRate(Constant::openLoopRampRate);
    driveTrain.driveMotor6.SetOpenLoopRampRate(Constant::openLoopRampRate);
}

void Drivetrain::Teleop(Drivetrain &driveTrain, Input &input, LimeLight &limelight) {
    double leftY = input.driveJoystick.GetY();
    double rightx = -input.driveJoystick.GetX();

    differentialDrive.ArcadeDrive(leftY, rightx);

    /*
    if (input.xboxController.GetRightTriggerAxis() > 0) {
       if (limelight.targetFound == 1) {
            if (limelight.targetOffsetHorizontal < -5) {
                driveTrain.differentialDrive.TankDrive(0.35, -0.35);
            } else if (limelight.targetOffsetHorizontal > 5) { 
                driveTrain.differentialDrive.TankDrive(-0.35, 0.35);
            } else {
                driveTrain.differentialDrive.TankDrive(0, 0);
            }
        } 
    } else if (input.xboxController.GetLeftTriggerAxis() > 0) {
        differentialDrive.ArcadeDrive(leftY * 0.5, rightY);
    } else {
        differentialDrive.ArcadeDrive(leftY, rightY);
    }
    */
}

void Drivetrain::Disable() {
    this->differentialDrive.TankDrive(0, 0);
}