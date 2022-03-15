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
    double leftY = input.xboxController.GetLeftY();
    double rightY = input.xboxController.GetRightY();

    if (input.xboxController.GetRightTriggerAxis() > 0) {
        if (limelight.targetFound == 1) {
            if (limelight.targetOffsetHorizontal < -3) {
                driveTrain.differentialDrive.TankDrive(0.35, -0.35);
            } else if (limelight.targetOffsetHorizontal > 3) { 
                driveTrain.differentialDrive.TankDrive(-0.35, 0.35);
            } else {
                driveTrain.differentialDrive.TankDrive(0, 0);
            }
        }
    } else if (input.xboxController.GetLeftTriggerAxis() >  0) {
        driveTrain.differentialDrive.TankDrive(leftY * 0.4, rightY * 0.4);
    } else {
        double speedLeft;
        double speedRight;

        if ((leftY < 0 && rightY > 0) || (leftY > 0 && rightY < 0)) {
            speedLeft = leftY * 0.65;
            speedRight = rightY * 0.65;
        } else {
            speedLeft = leftY * 0.9;
            speedRight = rightY * 0.9;
        }

        driveTrain.differentialDrive.TankDrive(speedLeft, speedRight);
    }
}

void Drivetrain::Disable() {
    this->differentialDrive.TankDrive(0, 0);
}