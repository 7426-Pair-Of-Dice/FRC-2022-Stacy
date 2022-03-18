#pragma once

#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc/PS4Controller.h>

class Input {
    public:
        //frc::XboxController xboxController{0};
        frc::Joystick driveJoystick{0};
        frc::Joystick joystick{1};
};