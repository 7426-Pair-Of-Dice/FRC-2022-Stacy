#include "../include/Intake.h"
#include "../include/Input.h"
#include "../include/Constant.h"

void Intake::Init(Intake &intake) {}

void Intake::Teleop(Intake &intake, Input &input) {
    if (input.joystick.GetRawButton(7)) {
        intake.intakeSolenoid.Set(intake.intakeSolenoid.kForward);
    } else {
        intake.intakeSolenoid.Set(intake.intakeSolenoid.kReverse);
    }

    if (input.joystick.GetRawButton(2)) {

        // Index in
        intake.indexMotorR.Set(Constant::intakePercentSpeed);
        intake.indexMotorL.Set(Constant::intakePercentSpeed);

        // Disable intake motor
        intake.intakeMotor.Set(0);

    } else if (input.joystick.GetRawButton(3)) {

        // Intake in 
        intake.intakeMotor.Set(Constant::intakePercentSpeed);
        
        // Index in
        intake.indexMotorR.Set(Constant::intakePercentSpeed);
        intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        
    } else if (input.joystick.GetRawButton(5)) {

        // Intake out 
        intake.intakeMotor.Set(-Constant::intakePercentSpeed);

        // Index out
        intake.indexMotorR.Set(-Constant::intakePercentSpeed);
        intake.indexMotorL.Set(-Constant::intakePercentSpeed);

    } else { // Disable all motors and solenoids while inactive

        intake.intakeMotor.Set(0);
        intake.indexMotorR.Set(0);
        intake.indexMotorL.Set(0);
    }
}

void Intake::Disable() {
    this->intakeSolenoid.Set(this->intakeSolenoid.kReverse);
    this->intakeMotor.Set(0);
    this->indexMotorR.Set(0);
    this->indexMotorL.Set(0);
}