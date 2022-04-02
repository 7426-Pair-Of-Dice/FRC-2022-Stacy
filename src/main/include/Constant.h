#pragma once

#include <frc/PneumaticsModuleType.h>

class Constant {
    public: 

        static constexpr double autoSpeeds[4][2] = {
            // {driveSpeed, turnSpeed}
            {0.66, 0.42}, // 2 ball
            {0.66, 0.42}, // 3 ball
            {0.90, 0.54}, // 4 ball-H
            {1, 0.5}      // 4 ball-R
        };

        static constexpr double intakePercentSpeed = 0.6;
        static constexpr double climbPercentSpeed = 1;
        static constexpr double openLoopRampRate = 0.85;
        static constexpr double flywheelTopGoalSpeed = 305;
        static constexpr double flywheelBottomGoalSpeed = 128;
        static constexpr double autoStraightDeadzone = 2;
        static constexpr double autoStraight = 0.08;
        static constexpr double autoStraightPConstant = 0.001;

        static constexpr double auto2BallFlywheelSpeed = 305;
        static constexpr double auto4BallFlywheelSpeed = 309;

        static constexpr double goalHeight = 104;
        static constexpr double limelightHeight = 41;
        static constexpr double limelightAngle = 45;

        static constexpr frc::PneumaticsModuleType pneumaticModule = frc::PneumaticsModuleType::CTREPCM;
};