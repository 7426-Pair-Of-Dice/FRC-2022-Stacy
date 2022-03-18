#include "../include/Shooter.h"
#include "../include/Input.h"

void Shooter::Init(Shooter &shooter) {
    shooter.shooterMotorL.SetInverted(true);

    shooter.shooterMotorL.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 30);
    shooter.shooterMotorR.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 30);

    shooter.shooterMotorL.SetSensorPhase(true);
    shooter.shooterMotorR.SetSensorPhase(true);

    shooter.shooterMotorL.ConfigNominalOutputForward(0, 30);
    shooter.shooterMotorL.ConfigNominalOutputReverse(0, 30);
    shooter.shooterMotorL.ConfigPeakOutputForward(1, 30);
    shooter.shooterMotorL.ConfigPeakOutputReverse(-1, 30);

    shooter.shooterMotorR.ConfigNominalOutputForward(0, 30);
    shooter.shooterMotorR.ConfigNominalOutputReverse(0, 30);
    shooter.shooterMotorR.ConfigPeakOutputForward(1, 30);
    shooter.shooterMotorR.ConfigPeakOutputReverse(-1, 30);

    shooter.shooterMotorL.Config_kF(0, 0.1097, 30);
    shooter.shooterMotorL.Config_kP(0, 0.22, 30);
    shooter.shooterMotorL.Config_kI(0, 0.0, 30);
    shooter.shooterMotorL.Config_kD(0, 0.0, 30);

    shooter.shooterMotorR.Config_kF(0, 0.1097, 30);
    shooter.shooterMotorR.Config_kP(0, 0.22, 30);
    shooter.shooterMotorR.Config_kI(0, 0.0, 30);
    shooter.shooterMotorR.Config_kD(0, 0.0, 30);
}

void Shooter::Teleop(Shooter &shooter, Input &input, LimeLight &limelight) {
    double targetVelocity;
    double shooterRPM = limelight.GetFlywheelSpeedFromDistance();

    if (input.joystick.GetRawButton(8) && input.joystick.GetRawButton(1)) {
        
        targetVelocity = Constant::flywheelBottomGoalSpeed * 4096 / 600;

        shooter.shooterMotorL.Set(ControlMode::Velocity, targetVelocity);
        shooter.shooterMotorR.Set(ControlMode::Velocity, targetVelocity);

    } else if (input.joystick.GetRawButton(1)) {
        targetVelocity = shooterRPM * 4096 / 600;

        shooter.shooterMotorL.Set(ControlMode::Velocity, targetVelocity);
        shooter.shooterMotorR.Set(ControlMode::Velocity, targetVelocity);
    } else {
        shooter.shooterMotorL.Set(ControlMode::PercentOutput, 0);
        shooter.shooterMotorR.Set(ControlMode::PercentOutput, 0);
    }

}

void Shooter::Disable() {
    this->shooterMotorL.Set(ControlMode::PercentOutput, 0);
    this->shooterMotorR.Set(ControlMode::PercentOutput, 0);
}