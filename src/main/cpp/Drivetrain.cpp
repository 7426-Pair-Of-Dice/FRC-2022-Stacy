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
    double rightY = -input.xboxController.GetRightX();

    //double p = -(0 - limelight.targetOffsetHorizontal) * 0.025;
    //double p = absolute(limelight.targetOffsetHorizontal) * 0.08;
    double speed = 0.25;
    //double rotation = speed;

    if (input.xboxController.GetRightTriggerAxis() > 0) {
        
        //driveTrain.differentialDrive.ArcadeDrive(0, rotation);
        
        if (limelight.targetFound == 1) {
            if (limelight.targetOffsetHorizontal < -2) {
                driveTrain.differentialDrive.TankDrive(speed, -speed);
            } else if (limelight.targetOffsetHorizontal > 2) { 
                driveTrain.differentialDrive.TankDrive(-speed, speed);
            } else {
                driveTrain.differentialDrive.TankDrive(0, 0);
            }
        } 
        
    } else if (input.xboxController.GetLeftTriggerAxis() > 0) {
        differentialDrive.ArcadeDrive(leftY * 0.5, rightY);
    } else {
        differentialDrive.ArcadeDrive(leftY, rightY);
    }
}

double Drivetrain::absolute(double x) {
    if (x < 0) return x * -1;
    if (x > 0) return x;
}

void Drivetrain::Disable() {
    this->differentialDrive.TankDrive(0, 0);
}