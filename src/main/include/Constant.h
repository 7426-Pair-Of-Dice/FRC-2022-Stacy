#pragma once

#include <frc/PneumaticsModuleType.h>

class Constant {
    public: 
        static constexpr double intakePercentSpeed = 0.5;
        static constexpr double climbPercentSpeed = 1;
        static constexpr double openLoopRampRate = 0.85;
        static constexpr double flywheelTopGoalSpeed = 305;
        static constexpr double flywheelBottomGoalSpeed = 128;

        static constexpr double autoDriveSpeed = 0.495;
        static constexpr double autoTurnSpeed = 0.405;
        static constexpr double autoStraight = 0.08;

        static constexpr double auto2BallFlywheelSpeed = 305;
        static constexpr double auto4BallFlywheelSpeed = 309;

        static constexpr double goalHeight = 104;
        static constexpr double limelightHeight = 41;
        static constexpr double limelightAngle = 13;

        static constexpr frc::PneumaticsModuleType pneumaticModule = frc::PneumaticsModuleType::CTREPCM;
};