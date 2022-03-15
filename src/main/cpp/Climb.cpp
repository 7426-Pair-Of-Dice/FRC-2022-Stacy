#include "../include/Climb.h"
#include "../include/Input.h"
#include "../include/Constant.h"

void Climb::Init(Climb &climb) {}

void Climb::Teleop(Climb &climb, Input &input) {

    if (input.joystick.GetRawButton(12)) {
        climb.climbArmSolenoid.Set(climb.climbArmSolenoid.kForward);
    } else if (input.joystick.GetRawButton(11)) {
        climb.climbArmSolenoid.Set(climb.climbArmSolenoid.kReverse);
    }

    if (input.joystick.GetRawButton(10)) {
        climb.climbHookSolenoid.Set(climb.climbHookSolenoid.kForward);
    } else if (input.joystick.GetRawButton(9)) {
        climb.climbHookSolenoid.Set(climb.climbHookSolenoid.kReverse);
    }

    if (input.joystick.GetRawButton(4)) {
        climb.lastPosition = climb.climbMotor.GetSelectedSensorPosition(0);
        climb.climbMotor.Set(ControlMode::PercentOutput, Constant::climbPercentSpeed);
    } else if (input.joystick.GetRawButton(6)) {
        climb.lastPosition = climb.climbMotor.GetSelectedSensorPosition(0);
        climb.climbMotor.Set(ControlMode::PercentOutput, -Constant::climbPercentSpeed);
    } else {
        climb.climbMotor.Set(ControlMode::Position, climb.lastPosition); 
    }
}

