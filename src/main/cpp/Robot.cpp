#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTable.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <cameraserver/CameraServer.h>
#include <frc/AnalogGyro.h> // Aidan this is for testing, this was not here before.
#include <frc/ADXRS450_Gyro.h>

#include <../include/Drivetrain.h>
#include <../include/Input.h>
#include <../include/Shooter.h>
#include <../include/Intake.h>
#include <../include/Climb.h>
#include <../include/LimeLight.h>
#include <../include/Constant.h>


class Robot : public frc::TimedRobot {

  private:
    Input input;
    Drivetrain driveTrain;
    Shooter shooter;
    Intake intake;
    Climb climb;
    LimeLight limelight;
    //AHRS ahrs{};
    frc::ADXRS450_Gyro mainGyro;
    
    bool autoPhase0Complete = false;
    bool autoPhase1Complete = false;
    bool autoPhase2Complete = false;
    bool autoPhase3Complete = false;
    bool autoPhase4Complete = false;
    bool autoPhase5Complete = false;
    bool autoPhase6Complete = false;
    bool autoPhase7Complete = false;
    bool autoPhase8Complete = false;
    bool autoPhase9Complete = false;
    bool autoPhase10Complete = false;
    bool autoPhase11Complete = false;
    bool autoPhase12Complete = false;
    bool autoPhase13Complete = false;
    bool autoPhase14Complete = false;
    bool autoPhase15Complete = false;
    bool autoPhase16Complete = false;

    double autoPhase0Timer = 0.0;
    double autoPhase2Timer = 0.0;
    double autoPhase3Timer = 0.0;
    double autoPhase6Timer = 0.0;
    double autoPhase7Timer = 0.0;
    double autoPhase9Timer = 0.0;
    double autoPhase10Timer = 0.0;
    double autoPhase11Timer = 0.0;
    double autoPhase12Timer = 0.0;
    double autoPhase13Timer = 0.0;
    double autoPhase14Timer = 0.0;
    double autoPhase15Timer = 0.0;

    double autoDriveSpeed;
    double autoTurnSpeed;

    frc::SendableChooser<std::string> m_chooser;
  
    const std::string kAutoNameDefault = "Default";
    const std::string kAutoNameCustom = "My Auto";
    std::string m_autoSelected;

  public:

    void RobotInit() override {
      frc::CameraServer::StartAutomaticCapture();

      mainGyro.Calibrate();
      driveTrain.Init(driveTrain);
      shooter.Init(shooter);
      intake.Init(intake);
      climb.Init(climb);

      m_chooser.SetDefaultOption("2-Ball", "2-Ball");
      m_chooser.AddOption("3-Ball", "3-Ball");
      m_chooser.AddOption("4-Ball-H", "4-Ball-H");
      m_chooser.AddOption("4-Ball-R", "4-Ball-R");
      m_chooser.AddOption("Gyro Test", "Gyro Test");
      m_chooser.AddOption("Straight Test", "Straight Test");
      frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
      
      frc::SmartDashboard::PutNumber("Shooter RPM", 320);
      frc::Shuffleboard::GetTab("Gyro").Add(mainGyro);

    };

    void RobotPeriodic() override {
      frc::SmartDashboard::PutNumber("Distance From Goal", limelight.GetDistanceFromTarget());
      /*
      frc::SmartDashboard::PutNumber("Yaw", ahrs.GetYaw());
      frc::SmartDashboard::PutNumber("Roll", ahrs.GetRoll());
      frc::SmartDashboard::PutNumber("Pitch", ahrs.GetPitch());
      */
      limelight.UpdateLimelightValues();
    };

    void AutonomousInit() override {
      limelight.EnableLEDs();

      m_autoSelected = m_chooser.GetSelected();

      if (m_autoSelected == "2-Ball") {
        autoDriveSpeed = Constant::autoSpeeds[0][0];
        autoTurnSpeed = Constant::autoSpeeds[0][1];
      } else if (m_autoSelected == "3-Ball") {
        autoDriveSpeed = Constant::autoSpeeds[1][0];
        autoTurnSpeed = Constant::autoSpeeds[1][1];
      } else if (m_autoSelected == "4-Ball-H") {
        autoDriveSpeed = Constant::autoSpeeds[2][0];
        autoTurnSpeed = Constant::autoSpeeds[2][1];
      } else if (m_autoSelected == "4-Ball-R") {
        autoDriveSpeed = Constant::autoSpeeds[3][0];
        autoTurnSpeed = Constant::autoSpeeds[3][1];
      }

      ResetAutonomous();
    };

    void AutonomousPeriodic() override {
      if (m_autoSelected == "2-Ball") { Autonomous1(); }
      else if (m_autoSelected == "3-Ball") { Autonomous2(); }
      else if (m_autoSelected == "4-Ball-H") { Autonomous3(); }
      else if (m_autoSelected == "4-Ball-R") { Autonomous4(); }
      else if (m_autoSelected == "Gyro Test") { AutonomousGyroTest(); }
      else if (m_autoSelected == "Straight Test") { AutonomousStraightTest(); }
      else { AutonomousGyroTest(); }
    };

    void TeleopInit() override {
      limelight.EnableLEDs();
    };

    void TeleopPeriodic() override {
      driveTrain.Teleop(driveTrain, input, limelight);
      shooter.Teleop(shooter, input, limelight);
      intake.Teleop(intake, input);
      climb.Teleop(climb, input);
    };

    void DisabledInit() override {
      driveTrain.Disable();
      intake.Disable();
      shooter.Disable();
      limelight.DisableLEDs();
      ResetAutonomous();
    };

    void DisabledPeriodic() override {};

    void TestInit() override {
      limelight.EnableLEDs();
    }

    void TestPeriodic() override {};

    void AutonomousStraightTest() {

      double positionLeft = driveTrain.leftDriveEncoder.GetPosition();
      double positionRight = -driveTrain.rightDriveEncoder.GetPosition();

      if (!autoPhase1Complete) {
        mainGyro.Reset();
        autoPhase1Complete = true;
      }
      
      if (autoPhase1Complete && !autoPhase2Complete) {
        double driveSetPoint = 60;
        double speedIncrement = abs(mainGyro.GetAngle()) * 0.015;

        if (positionLeft < driveSetPoint || positionRight < driveSetPoint) {
          if (mainGyro.GetAngle() > Constant::autoStraightDeadzone) { // Too Far Right
            driveTrain.differentialDrive.TankDrive(-autoDriveSpeed, -(autoDriveSpeed + speedIncrement));
          } else if (mainGyro.GetAngle() < -Constant::autoStraightDeadzone) { // Too Far Left
            driveTrain.differentialDrive.TankDrive(-(autoDriveSpeed + speedIncrement), -autoDriveSpeed);
          } else { // Just Right
            driveTrain.differentialDrive.TankDrive(-autoDriveSpeed, -autoDriveSpeed);
          }
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          mainGyro.Reset();
          autoPhase2Complete = true;
        }
      }

      if (autoPhase2Complete) {
        driveTrain.differentialDrive.TankDrive(0, 0);
      }
    }

    void AutonomousGyroTest() {
      if(!autoPhase1Complete) {
        mainGyro.Reset();
        autoPhase1Complete = true;
      }


      if (autoPhase1Complete && !autoPhase2Complete) {
        double degrees = -4.2; // Why do we die

        if (mainGyro.GetAngle() > degrees) {
          driveTrain.differentialDrive.TankDrive(-autoTurnSpeed, autoTurnSpeed);
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          autoPhase2Complete = true;
          mainGyro.Reset();
        }
      }

      if (autoPhase2Complete && !autoPhase3Complete) {
        double degrees = 2.1; // Why do we die

        if (mainGyro.GetAngle() < degrees) {
          driveTrain.differentialDrive.TankDrive(autoTurnSpeed, -autoTurnSpeed);
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          autoPhase3Complete = true;
          mainGyro.Reset();
        }
      }

      if (autoPhase3Complete && !autoPhase4Complete) {
        double degrees = -2.1; // Why do we die

        if (mainGyro.GetAngle() > degrees) {
          driveTrain.differentialDrive.TankDrive(-autoTurnSpeed, autoTurnSpeed);
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          autoPhase4Complete = true;
          mainGyro.Reset();
        }
      }

      if (autoPhase4Complete) { // This is dangerous
        driveTrain.differentialDrive.TankDrive(0, 0);
      }

    }

    void Autonomous1() {
      
      /*
      2-Ball Autonomous
      Drive Speed 0.66
      Turn Speed 0.42
      */

      double positionLeft = driveTrain.leftDriveEncoder.GetPosition();
      double positionRight = -driveTrain.rightDriveEncoder.GetPosition();

      if (!autoPhase0Complete) {
        autoPhase0Timer = autoPhase0Timer + 20;

        if (autoPhase0Timer > 200) {
          autoPhase0Complete = true;
        }
      }

      /*
        Auto Phase 1
        Moves robot forward to a set point.
        Moves towards first ball.
      */
      if (autoPhase0Complete && !autoPhase1Complete) {
        double driveSetPoint = 18;
        double speedIncrement = abs(mainGyro.GetAngle())*Constant::autoStraightPConstant;

        intake.intakeSolenoid.Set(intake.intakeSolenoid.kForward);

        intake.intakeMotor.Set(Constant::intakePercentSpeed);
        intake.indexMotorR.Set(Constant::intakePercentSpeed);
        intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        
        if (positionLeft < driveSetPoint || positionRight < driveSetPoint) {
          if (mainGyro.GetAngle() > Constant::autoStraightDeadzone) { // Too Far Right
            driveTrain.differentialDrive.TankDrive(-autoDriveSpeed, -(autoDriveSpeed + speedIncrement));
          } else if (mainGyro.GetAngle() < -Constant::autoStraightDeadzone) { // Too Far Left
            driveTrain.differentialDrive.TankDrive(-(autoDriveSpeed + speedIncrement), -autoDriveSpeed);
          } else { // Just Right
            driveTrain.differentialDrive.TankDrive(-autoDriveSpeed, -autoDriveSpeed);
          }
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);

          //intake.intakeSolenoid.Set(intake.intakeSolenoid.kReverse);
          //intake.intakeMotor.Set(0);
          //intake.indexMotorR.Set(0);
          //intake.indexMotorL.Set(0);
          intake.intakeMotor.Set(Constant::intakePercentSpeed);
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(-Constant::intakePercentSpeed);
          mainGyro.Reset();

          autoPhase1Complete = true;
        }
      } 

      /*
        Auto Phase 2
        Runs intake motor and index motors.
        Pulls ball in.
      */
      if (autoPhase1Complete && !autoPhase2Complete) {
        autoPhase2Timer = autoPhase2Timer + 20;

        if (autoPhase2Timer < 650) {
    
          intake.intakeMotor.Set(Constant::intakePercentSpeed);
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        } else {
          
          intake.intakeMotor.Set(0);
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);
          mainGyro.Reset();

          autoPhase2Complete = true;
        }
      }

      /*
        Auto Phase 3
        Runs index motors.
        Indexes ball further.
      */
      if (autoPhase2Complete && !autoPhase3Complete) {
        autoPhase3Timer = autoPhase3Timer + 20;
        intake.intakeSolenoid.Set(intake.intakeSolenoid.kReverse);
        if (autoPhase3Timer < 650) {
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        } else {
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);
          
          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);
          mainGyro.Reset();

          autoPhase3Complete = true;
        }
      }

      /*
        Auto Phase 4
        Turns robot X degrees to the right.
        Turns robot towards goal.
      */
      if (autoPhase3Complete && !autoPhase4Complete) {
        double degrees = 155; // Why do we die

        if (mainGyro.GetAngle() < degrees) {
          driveTrain.differentialDrive.TankDrive(-autoTurnSpeed, autoTurnSpeed);
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          autoPhase4Complete = true;
          mainGyro.Reset();
        }
      }

      /*
        Auto Phase 5
        Turns to focus limelight.
      */
      if (autoPhase4Complete && !autoPhase5Complete) {
        if (limelight.targetFound == 1) {
            if (limelight.targetOffsetHorizontal < -2) {
                driveTrain.differentialDrive.TankDrive(0.25, -0.25);
            } else if (limelight.targetOffsetHorizontal > 2) {
                driveTrain.differentialDrive.TankDrive(-0.25, 0.25);
            } else {
              driveTrain.differentialDrive.TankDrive(0, 0);

              autoPhase5Complete = true;
            }
        }
      }

      /*
        Auto Phase 6
        Shoots ball at X RPM.
        Shoot two balls into the goal.
      */
      if (autoPhase5Complete && !autoPhase6Complete) {
        autoPhase6Timer = autoPhase6Timer + 20;

        if (autoPhase6Timer < 1000) {
          double shooterRPM = limelight.GetFlywheelSpeedFromDistance();
          double targetVelocity = shooterRPM * 4096 / 600;

          shooter.shooterMotorL.Set(ControlMode::Velocity, targetVelocity);
          shooter.shooterMotorR.Set(ControlMode::Velocity, targetVelocity);
        } else if (autoPhase6Timer < 4000) {
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(Constant::intakePercentSpeed);
          intake.intakeMotor.Set(Constant::intakePercentSpeed);
        } else {
          intake.intakeMotor.Set(0);
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);
          shooter.shooterMotorL.Set(ControlMode::PercentOutput, 0);
          shooter.shooterMotorR.Set(ControlMode::PercentOutput, 0);

          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);
          mainGyro.Reset();

          autoPhase6Complete = true;
        }
      }

      // Autonomous Complete
      if (autoPhase6Complete) {
        driveTrain.differentialDrive.TankDrive(0, 0);
      }
    };

    void Autonomous2() {
      
      /*
      3-Ball Autonomous
      Drive Speed 0.66
      Turn Speed 0.42
      */

      double positionLeft = driveTrain.leftDriveEncoder.GetPosition();
      double positionRight = -driveTrain.rightDriveEncoder.GetPosition();

      if (!autoPhase0Complete) {
        autoPhase0Timer = autoPhase0Timer + 20;

        if (autoPhase0Timer > 200) {
          autoPhase0Complete = true;
        }
      }

      /*
        Auto Phase 1
        Moves robot forward to a set point.
        Moves towards first ball.
      */
      if (autoPhase0Complete && !autoPhase1Complete) {
        double driveSetPoint = 18;
        double speedIncrement = abs(mainGyro.GetAngle())*Constant::autoStraightPConstant;

        intake.intakeSolenoid.Set(intake.intakeSolenoid.kForward);

        intake.intakeMotor.Set(Constant::intakePercentSpeed);
        intake.indexMotorR.Set(Constant::intakePercentSpeed);
        intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        
        if (positionLeft < driveSetPoint || positionRight < driveSetPoint) {
          if (mainGyro.GetAngle() > Constant::autoStraightDeadzone) { // Too Far Right
            driveTrain.differentialDrive.TankDrive(-autoDriveSpeed, -(autoDriveSpeed + speedIncrement));
          } else if (mainGyro.GetAngle() < -Constant::autoStraightDeadzone) { // Too Far Left
            driveTrain.differentialDrive.TankDrive(-(autoDriveSpeed + speedIncrement), -autoDriveSpeed);
          } else { // Just Right
            driveTrain.differentialDrive.TankDrive(-autoDriveSpeed, -autoDriveSpeed);
          }
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);

          //intake.intakeSolenoid.Set(intake.intakeSolenoid.kReverse);
          //intake.intakeMotor.Set(0);
          //intake.indexMotorR.Set(0);
          //intake.indexMotorL.Set(0);
          intake.intakeMotor.Set(Constant::intakePercentSpeed);
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(-Constant::intakePercentSpeed);
          mainGyro.Reset();

          autoPhase1Complete = true;
        }
      } 

      /*
        Auto Phase 2
        Runs intake motor and index motors.
        Pulls ball in.
      */
      if (autoPhase1Complete && !autoPhase2Complete) {
        autoPhase2Timer = autoPhase2Timer + 20;

        if (autoPhase2Timer < 650) {
    
          intake.intakeMotor.Set(Constant::intakePercentSpeed);
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        } else {
          
          intake.intakeMotor.Set(0);
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);
          mainGyro.Reset();

          autoPhase2Complete = true;
        }
      }

      /*
        Auto Phase 3
        Runs index motors.
        Indexes ball further.
      */
      if (autoPhase2Complete && !autoPhase3Complete) {
        autoPhase3Timer = autoPhase3Timer + 20;
        intake.intakeSolenoid.Set(intake.intakeSolenoid.kReverse);
        if (autoPhase3Timer < 650) {
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        } else {
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);
          
          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);
          mainGyro.Reset();

          autoPhase3Complete = true;
        }
      }

      /*
        Auto Phase 4
        Turns robot X degrees to the right.
        Turns robot towards goal.
      */
      if (autoPhase3Complete && !autoPhase4Complete) {
        double degrees = 173; // Why do we die

        if (mainGyro.GetAngle() < degrees) {
          driveTrain.differentialDrive.TankDrive(-autoTurnSpeed, autoTurnSpeed);
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          autoPhase4Complete = true;
          autoPhase5Complete = true;
          mainGyro.Reset();
        }
      }

      /*
        Auto Phase 5
        Turns to focus limelight.
        DISABLED
      */
      if (autoPhase4Complete && !autoPhase5Complete) {
        if (limelight.targetFound == 1) {
            if (limelight.targetOffsetHorizontal < -5) {
                driveTrain.differentialDrive.TankDrive(autoTurnSpeed, -autoTurnSpeed);
            } else if (limelight.targetOffsetHorizontal > 5) {
                driveTrain.differentialDrive.TankDrive(-autoTurnSpeed, autoTurnSpeed);
            } else {
              driveTrain.differentialDrive.TankDrive(0, 0);

              autoPhase5Complete = true;
            }
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          mainGyro.Reset();

          autoPhase5Complete = true;
        }
      }

      /*
        Auto Phase 6
        Shoots ball at X RPM.
        Shoot two balls into the goal.
      */
      if (autoPhase5Complete && !autoPhase6Complete) {
        autoPhase6Timer = autoPhase6Timer + 20;

        if (autoPhase6Timer < 1000) {
          double shooterRPM = 323;
          double targetVelocity = Constant::auto4BallFlywheelSpeed * 4096 / 600;

          shooter.shooterMotorL.Set(ControlMode::Velocity, targetVelocity);
          shooter.shooterMotorR.Set(ControlMode::Velocity, targetVelocity);
        } else if (autoPhase6Timer < 1800) {
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(Constant::intakePercentSpeed);
          intake.intakeMotor.Set(Constant::intakePercentSpeed);
        } else {
          intake.intakeMotor.Set(0);
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);
          shooter.shooterMotorL.Set(ControlMode::PercentOutput, 0);
          shooter.shooterMotorR.Set(ControlMode::PercentOutput, 0);

          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);
          mainGyro.Reset();

          autoPhase6Complete = true;
        }
      }
      
      /*
        Auto Phase 7
        Turns robot X degrees to the left.
        Turns robot towards second ball.
      */
      if (autoPhase6Complete && !autoPhase7Complete) {
        double degrees = -45; // Why do we die

        if (mainGyro.GetAngle() > degrees) {
          driveTrain.differentialDrive.TankDrive(autoTurnSpeed, -autoTurnSpeed);
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          mainGyro.Reset();
          autoPhase7Complete = true;
        }
        
      }
      
      /*
        Auto Phase 8
        Moves robot forward to a set point.
        Moves towards second ball.
      */
      if (autoPhase7Complete && !autoPhase8Complete) {
        double driveSetPoint = 57;
        double speedIncrement = abs(mainGyro.GetAngle())*Constant::autoStraightPConstant;

        intake.intakeSolenoid.Set(intake.intakeSolenoid.kForward);

        intake.intakeMotor.Set(Constant::intakePercentSpeed);
        intake.indexMotorR.Set(Constant::intakePercentSpeed);
        intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        
        if (positionLeft < driveSetPoint || positionRight < driveSetPoint) {
          if (mainGyro.GetAngle() > Constant::autoStraightDeadzone) { // Too Far Right
            driveTrain.differentialDrive.TankDrive(-autoDriveSpeed, -(autoDriveSpeed + speedIncrement));
          } else if (mainGyro.GetAngle() < -Constant::autoStraightDeadzone) { // Too Far Left
            driveTrain.differentialDrive.TankDrive(-(autoDriveSpeed + speedIncrement), -autoDriveSpeed);
          } else { // Just Right
            driveTrain.differentialDrive.TankDrive(-autoDriveSpeed, -autoDriveSpeed);
          }
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          
          // intake.intakeSolenoid.Set(intake.intakeSolenoid.kReverse);
          //intake.intakeMotor.Set(0);
          //intake.indexMotorR.Set(0);
          //intake.indexMotorL.Set(0);
          intake.intakeMotor.Set(Constant::intakePercentSpeed);
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(-Constant::intakePercentSpeed);
          
          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);

          autoPhase8Complete = true;
        }
      }

      /*
        Auto Phase 9
        Runs intake motor and index motors.
        Pulls ball in.
      */
      if (autoPhase8Complete && !autoPhase9Complete) {
        autoPhase9Timer = autoPhase9Timer + 20;

        if (autoPhase9Timer < 600) {
    
          intake.intakeMotor.Set(Constant::intakePercentSpeed);
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        } else {
          
          intake.intakeMotor.Set(0);
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);
          mainGyro.Reset();

          autoPhase9Complete = true;
        }
      }

      /*
        Auto Phase 10
        Runs index motors.
        Indexes ball further.
      */
      if (autoPhase9Complete && !autoPhase10Complete) {
        autoPhase10Timer = autoPhase10Timer + 20;
        intake.intakeSolenoid.Set(intake.intakeSolenoid.kReverse);
        if (autoPhase10Timer < 500) {
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        } else {
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);
          
          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);
          mainGyro.Reset();

          autoPhase10Complete = true;
        }
      }

      /*
        Auto Phase 11
        Turns robot X degrees to the right.
        Turns robot towards goal.
      */
      if (autoPhase10Complete && !autoPhase11Complete) { //Phase 9
        double degrees = 75; // Why do we die

        if (mainGyro.GetAngle() < degrees) {
          driveTrain.differentialDrive.TankDrive(-autoTurnSpeed, autoTurnSpeed);
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          autoPhase11Complete = true;
          mainGyro.Reset();
        }
      }

      if (autoPhase11Complete && !autoPhase12Complete) {
        if (limelight.targetFound == 1) {
            if (limelight.targetOffsetHorizontal < -2) {
                driveTrain.differentialDrive.TankDrive(0.25, -0.25);
            } else if (limelight.targetOffsetHorizontal > 2) {
                driveTrain.differentialDrive.TankDrive(-0.25, 0.25);
            } else {
              driveTrain.differentialDrive.TankDrive(0, 0);

              autoPhase12Complete = true;
            }
        }
      }

      /*
        Auto Phase 12
        Shoots ball at X RPM.
        Shoot one ball into the goal.
      */
      if (autoPhase12Complete && !autoPhase13Complete) {
        autoPhase12Timer = autoPhase12Timer + 20;

        if (autoPhase12Timer < 1000) {
          double shooterRPM = limelight.GetFlywheelSpeedFromDistance();
          double targetVelocity = shooterRPM * 4096 / 600;

          shooter.shooterMotorL.Set(ControlMode::Velocity, targetVelocity);
          shooter.shooterMotorR.Set(ControlMode::Velocity, targetVelocity);
        } else if (autoPhase12Timer < 1800) {
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(Constant::intakePercentSpeed);
        } else {
          intake.intakeMotor.Set(0);
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);
          shooter.shooterMotorL.Set(ControlMode::PercentOutput, 0);
          shooter.shooterMotorR.Set(ControlMode::PercentOutput, 0);

          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);

          autoPhase13Complete = true;
        }
      }

      // Autonomous Complete
      if (autoPhase13Complete) {
        driveTrain.differentialDrive.TankDrive(0, 0);
      }
    }

    void Autonomous3() {

      /*
      4-Ball-H Autonomous
      Drive Speed 0.87
      Turn Speed 0.46
      */

      double positionLeft = driveTrain.leftDriveEncoder.GetPosition();
      double positionRight = -driveTrain.rightDriveEncoder.GetPosition();

      if (!autoPhase0Complete) {
        autoPhase0Timer = autoPhase0Timer + 20;

        if (autoPhase0Timer > 200) {
          autoPhase0Complete = true;
        }
      }

      /*
        Auto Phase 1
        Moves robot forward to a set point.
        Moves towards first ball.
      */

      if (autoPhase0Complete && !autoPhase1Complete) 
      {
        double driveSetPoint = 18;
        double speedIncrement = abs(mainGyro.GetAngle())*Constant::autoStraightPConstant;

        intake.intakeSolenoid.Set(intake.intakeSolenoid.kForward);

        
        intake.indexMotorR.Set(Constant::intakePercentSpeed);
        intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        intake.intakeMotor.Set(Constant::intakePercentSpeed);

        if (positionLeft < driveSetPoint || positionRight < driveSetPoint) {
          if (mainGyro.GetAngle() > Constant::autoStraightDeadzone) { // Too Far Right
            driveTrain.differentialDrive.TankDrive(-autoDriveSpeed, -(autoDriveSpeed + speedIncrement));
          } else if (mainGyro.GetAngle() < -Constant::autoStraightDeadzone) { // Too Far Left
            driveTrain.differentialDrive.TankDrive(-(autoDriveSpeed + speedIncrement), -autoDriveSpeed);
          } else { // Just Right
            driveTrain.differentialDrive.TankDrive(-autoDriveSpeed, -autoDriveSpeed);
          }
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
        
        intake.indexMotorR.Set(Constant::intakePercentSpeed);
        intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        intake.intakeMotor.Set(Constant::intakePercentSpeed);
          //intake.intakeSolenoid.Set(intake.intakeSolenoid.kReverse);
          //intake.intakeMotor.Set(0);
          //intake.indexMotorR.Set(0);
          //intake.indexMotorL.Set(0);
          mainGyro.Reset();

          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);
          
          autoPhase1Complete = true;
        }
      } 


      /*
        Auto Phase 2
        Runs intake motor and index motors.
        Pulls ball in.
      */

      if (autoPhase1Complete && !autoPhase2Complete) {
        /*
        Add a delay because of robot momentum
        */
        autoPhase2Timer = autoPhase2Timer + 20;

        if (autoPhase2Timer < 600) {
          intake.intakeMotor.Set(Constant::intakePercentSpeed);
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        } else {
         // intake.intakeMotor.Set(0);
         // intake.indexMotorR.Set(0);
          //intake.indexMotorL.Set(0);
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(-Constant::intakePercentSpeed);
          intake.intakeMotor.Set(Constant::intakePercentSpeed);
          mainGyro.Reset();

          autoPhase2Complete = true;
        }
      }

      /*
        Auto Phase 3
        Runs index motors.
        Indexes ball further.
      */

      if (autoPhase2Complete && !autoPhase3Complete) 
      {
        autoPhase3Timer = autoPhase3Timer + 20;

        intake.intakeSolenoid.Set(intake.intakeSolenoid.kReverse);

        if (autoPhase3Timer < 200)
        {
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        } else 
        {
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);

          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);
          mainGyro.Reset();

          autoPhase3Complete = true;
        }
      }

      /*
        Auto Phase 4
        Turns robot X degrees to the right.
        Turns robot towards goal.
      */

      if (autoPhase3Complete && !autoPhase4Complete) {
        double degrees = 140; // Why do we die: drugs

        if (mainGyro.GetAngle() < degrees) {
          driveTrain.differentialDrive.TankDrive(-autoTurnSpeed, autoTurnSpeed);
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          autoPhase4Complete = true;
          autoPhase5Complete = true;
          mainGyro.Reset();
        }
      }

      /*
        Auto Phase 5
        Turns to focus limelight.
        DISABLED
      */
      if (autoPhase4Complete && !autoPhase5Complete) {
        if (limelight.targetFound == 1) {
            if (limelight.targetOffsetHorizontal < -5) {
                driveTrain.differentialDrive.TankDrive(autoTurnSpeed, -autoTurnSpeed);
            } else if (limelight.targetOffsetHorizontal > 5) {
                driveTrain.differentialDrive.TankDrive(-autoTurnSpeed, autoTurnSpeed);
            } else {
              driveTrain.differentialDrive.TankDrive(0, 0);

              autoPhase5Complete = true;
            }
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          mainGyro.Reset();

          autoPhase5Complete = true;
        }
      }

      /*
        Auto Phase 6
        Shoots ball at X RPM.
        Shoot two balls into the goal.
      */

      if (autoPhase5Complete && !autoPhase6Complete) 
      {
        autoPhase6Timer = autoPhase6Timer + 20;

        if (autoPhase6Timer < 1000) 
        {
          double shooterRPM = 323;
          double targetVelocity = Constant::auto4BallFlywheelSpeed * 4096 / 600;

          shooter.shooterMotorL.Set(ControlMode::Velocity, targetVelocity);
          shooter.shooterMotorR.Set(ControlMode::Velocity, targetVelocity);
        } else if (autoPhase6Timer < 1800) {
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(Constant::intakePercentSpeed);
        } else 
        {
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);
          shooter.shooterMotorL.Set(ControlMode::PercentOutput, 0);
          shooter.shooterMotorR.Set(ControlMode::PercentOutput, 0);

          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);
          mainGyro.Reset();

          autoPhase6Complete = true;
        }
      }

      /*
        THE FIRST LEFT TURN
        Auto Phase 7
        Turns robot X degrees to the left.
        Turns robot towards second ball.
      */

      if (autoPhase6Complete && !autoPhase7Complete) {

        double degrees = -45; // Why do we die

        if (mainGyro.GetAngle() > degrees) {
          driveTrain.differentialDrive.TankDrive(autoTurnSpeed, -autoTurnSpeed);
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          mainGyro.Reset();

          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);

          autoPhase7Complete = true;
        }

      }
     
      /*
        Auto Phase 8
        Moves robot forward to a set point to intake a ball.
        Moves towards second ball with intake.
      */

      if (autoPhase7Complete && !autoPhase8Complete) {
        double driveSetPoint = 20;//22.75

        double speedIncrement = abs(mainGyro.GetAngle()) * Constant::autoStraightPConstant;

        intake.intakeSolenoid.Set(intake.intakeSolenoid.kForward);

        intake.intakeMotor.Set(Constant::intakePercentSpeed);
        intake.indexMotorR.Set(Constant::intakePercentSpeed);
        intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        
        if (positionLeft < driveSetPoint || positionRight < driveSetPoint) {
          if (mainGyro.GetAngle() > Constant::autoStraightDeadzone) { // Too Far Right
            driveTrain.differentialDrive.TankDrive(-autoDriveSpeed, -(autoDriveSpeed + speedIncrement));
          } else if (mainGyro.GetAngle() < -Constant::autoStraightDeadzone) { // Too Far Left
            driveTrain.differentialDrive.TankDrive(-(autoDriveSpeed + speedIncrement), -autoDriveSpeed);
          } else { // Just Right
            driveTrain.differentialDrive.TankDrive(-autoDriveSpeed, -autoDriveSpeed);
          }
        } else
        {
          driveTrain.differentialDrive.TankDrive(0, 0);
          
          intake.intakeSolenoid.Set(intake.intakeSolenoid.kReverse);
          intake.intakeMotor.Set(0);
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);

          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);
          mainGyro.Reset();
          autoPhase8Complete = true;
        }
      }

      /*
  
        Auto Phase 9
        Turns robot X degrees to the left.
        Turns robot towards second ball.
      */

      if (autoPhase8Complete && !autoPhase9Complete) 
      {
        double degrees = -1.5; // Why do we die

        if (mainGyro.GetAngle() > degrees) {
          driveTrain.differentialDrive.TankDrive(autoTurnSpeed, -autoTurnSpeed);
        } else {
          
          
          driveTrain.differentialDrive.TankDrive(0, 0);
          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);
          mainGyro.Reset();
          autoPhase9Complete = true;
          
        }
      }

      /*
        Auto Phase 10
        Moves robot forward to a set point to intake a ball.
        Moves towards second ball with intake.
      */

      if (autoPhase9Complete && !autoPhase10Complete) {
        autoPhase10Timer = autoPhase10Timer + 20;

        intake.intakeSolenoid.Set(intake.intakeSolenoid.kForward);

        intake.intakeMotor.Set(Constant::intakePercentSpeed);
        intake.indexMotorR.Set(Constant::intakePercentSpeed);
        intake.indexMotorL.Set(-Constant::intakePercentSpeed);

        if (autoPhase10Timer < 420) {
          driveTrain.differentialDrive.TankDrive(-autoDriveSpeed, -autoDriveSpeed + 0.42);
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          autoPhase10Complete = true;
        }
      } 
      /*
        Auto Phase 11
        Set Intake out for Human Player
      */
     
       if (autoPhase10Complete && !autoPhase11Complete) {
        
        autoPhase9Timer = autoPhase9Timer + 20;

        if (autoPhase9Timer < 1000) {
          intake.intakeSolenoid.Set(intake.intakeSolenoid.kReverse);

          intake.intakeMotor.Set(Constant::intakePercentSpeed);
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        } else if (autoPhase9Timer < 3000) { 
          intake.intakeSolenoid.Set(intake.intakeSolenoid.kForward);

          intake.intakeMotor.Set(Constant::intakePercentSpeed);
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        } else {
          intake.intakeSolenoid.Set(intake.intakeSolenoid.kReverse);

          //intake.intakeMotor.Set(0);
          //intake.indexMotorR.Set(0);
          //intake.indexMotorL.Set(0);

          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);
          mainGyro.Reset();

          autoPhase11Complete = true;
        }
      }
      
      /*
        Auto Phase 10
        Turns robot X degrees to the right.
        Turns robot towards goal.

        Change to a left
      */

      if (autoPhase11Complete && !autoPhase12Complete) 
      {
        
        double degrees = -70; // Why do we die

        if (mainGyro.GetAngle() > degrees) {
          driveTrain.differentialDrive.TankDrive(autoTurnSpeed, -autoTurnSpeed);
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
           driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);
          mainGyro.Reset();
          autoPhase12Complete = true;
        }
        
      }

      /*
        Auto Phase 11
        Robot Drives towards the goal 
      */


      if (autoPhase12Complete && !autoPhase13Complete) {
        double driveSetPoint = 27;
        double speedIncrement = abs(mainGyro.GetAngle())*Constant::autoStraightPConstant;

        intake.intakeSolenoid.Set(intake.intakeSolenoid.kForward);

        
        intake.indexMotorR.Set(Constant::intakePercentSpeed);
        intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        intake.intakeMotor.Set(Constant::intakePercentSpeed);

        if (positionLeft < driveSetPoint || positionRight < driveSetPoint) {
          if (mainGyro.GetAngle() > Constant::autoStraightDeadzone) { // Too Far Right
            driveTrain.differentialDrive.TankDrive(-autoDriveSpeed, -(autoDriveSpeed + speedIncrement));
          } else if (mainGyro.GetAngle() < -Constant::autoStraightDeadzone) { // Too Far Left
            driveTrain.differentialDrive.TankDrive(-(autoDriveSpeed + speedIncrement), -autoDriveSpeed);
          } else { // Just Right
            driveTrain.differentialDrive.TankDrive(-autoDriveSpeed, -autoDriveSpeed);
          }
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);

          intake.intakeSolenoid.Set(intake.intakeSolenoid.kReverse);
          intake.intakeMotor.Set(0);
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);
          mainGyro.Reset();

          autoPhase13Complete = true;
        }
      } 

      /*
        Auto Phase 12
        Robot Shoots 2 
      */

      if (autoPhase13Complete && !autoPhase14Complete) {
        if (limelight.targetFound == 1) {
            if (limelight.targetOffsetHorizontal < -5) {
                driveTrain.differentialDrive.TankDrive(autoTurnSpeed, -autoTurnSpeed);
            } else if (limelight.targetOffsetHorizontal > 5) {
                driveTrain.differentialDrive.TankDrive(-autoTurnSpeed, autoTurnSpeed);
            } else {
              driveTrain.differentialDrive.TankDrive(0, 0);

              autoPhase14Complete = true;
            }
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          mainGyro.Reset();

          autoPhase14Complete = true;
        }
      }


      if (autoPhase14Complete && !autoPhase15Complete) {
        autoPhase14Timer = autoPhase14Timer + 20;

        if (autoPhase14Timer < 1000) {
          double shooterRPM = 323;
          double targetVelocity = limelight.GetFlywheelSpeedFromDistance() * 4096 / 600;

          shooter.shooterMotorL.Set(ControlMode::Velocity, targetVelocity);
          shooter.shooterMotorR.Set(ControlMode::Velocity, targetVelocity);
        } else if (autoPhase14Timer < 2000) {
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(Constant::intakePercentSpeed);
        } else {
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);
          shooter.shooterMotorL.Set(ControlMode::PercentOutput, 0);
          shooter.shooterMotorR.Set(ControlMode::PercentOutput, 0);

          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);
          mainGyro.Reset();

          autoPhase15Complete = true;
        }
      }

      // Autonomous Complete
      if (autoPhase15Complete) {
        driveTrain.differentialDrive.TankDrive(0, 0);
      }
    }

    void Autonomous4() {
      
      /*
      4-Ball-R Autonomous
      Drive Speed 1.00
      Turn Speed 0.5
      */

      double positionLeft = driveTrain.leftDriveEncoder.GetPosition();
      double positionRight = -driveTrain.rightDriveEncoder.GetPosition();

      if (!autoPhase0Complete) {
        autoPhase0Timer = autoPhase0Timer + 20;

        if (autoPhase0Timer > 200) {
          autoPhase0Complete = true;
        }
      }

      /*
        Auto Phase 1
        Moves robot forward to a set point.
        Moves towards first ball.
      */
      
      if (autoPhase0Complete && !autoPhase1Complete) {
        double driveSetPoint = 9.5;
        double speedIncrement = abs(mainGyro.GetAngle()) * Constant::autoStraightPConstant;

        intake.intakeSolenoid.Set(intake.intakeSolenoid.kForward);

        intake.intakeMotor.Set(Constant::intakePercentSpeed);
        intake.indexMotorR.Set(Constant::intakePercentSpeed);
        intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        
        if (positionLeft < driveSetPoint || positionRight < driveSetPoint) {
          if (mainGyro.GetAngle() > Constant::autoStraightDeadzone) { // Too Far Right
            driveTrain.differentialDrive.TankDrive(-autoDriveSpeed, -(autoDriveSpeed + speedIncrement));
          } else if (mainGyro.GetAngle() < -Constant::autoStraightDeadzone) { // Too Far Left
            driveTrain.differentialDrive.TankDrive(-(autoDriveSpeed + speedIncrement), -autoDriveSpeed);
          } else { // Just Right
            driveTrain.differentialDrive.TankDrive(-autoDriveSpeed, -autoDriveSpeed);
          }
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);

          //intake.intakeSolenoid.Set(intake.intakeSolenoid.kReverse);
          //intake.intakeMotor.Set(0);
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);
          intake.intakeSolenoid.Set(intake.intakeSolenoid.kReverse);
          intake.intakeMotor.Set(0);
          mainGyro.Reset();

          autoPhase1Complete = true;
        }
      } 

      /*
        Auto Phase 2
        Runs intake motor and index motors.
        Pulls ball in.
      */
     
      if (autoPhase1Complete && !autoPhase2Complete) {
        autoPhase2Timer = autoPhase2Timer + 20;


        if (autoPhase2Timer < 600) {
          intake.intakeMotor.Set(Constant::intakePercentSpeed);
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        } else {
          intake.intakeMotor.Set(0);
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);
          mainGyro.Reset();

          autoPhase2Complete = true;
        }
      }

      /*
        Auto Phase 3
        Runs index motors.
        Indexes ball further.
      */
     
      if (autoPhase2Complete && !autoPhase3Complete) {
        autoPhase3Timer = autoPhase3Timer + 20;

        if (autoPhase3Timer < 100) {
          intake.indexMotorR.Set(-Constant::intakePercentSpeed);
          intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        } else {
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);

          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);
          mainGyro.Reset();

          autoPhase3Complete = true;
        }
      }

      /*
        Auto Phase 4
        Turns robot X degrees to the right.
        Turns robot towards goal.
      */
      if (autoPhase3Complete && !autoPhase4Complete) {
        double degrees = 170; // Why do we die

        if (mainGyro.GetAngle() < degrees) {
          driveTrain.differentialDrive.TankDrive(-autoTurnSpeed, autoTurnSpeed);
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          autoPhase4Complete = true;
          autoPhase5Complete = true;
          mainGyro.Reset();
        }
      }

      /*
        Auto Phase 5
        Turns to focus limelight.
        DISABLED
      */
      if (autoPhase4Complete && !autoPhase5Complete) {
        if (limelight.targetFound == 1) {
            if (limelight.targetOffsetHorizontal < -5) {
                driveTrain.differentialDrive.TankDrive(autoTurnSpeed, -autoTurnSpeed);
            } else if (limelight.targetOffsetHorizontal > 5) {
                driveTrain.differentialDrive.TankDrive(-autoTurnSpeed, autoTurnSpeed);
            } else {
              driveTrain.differentialDrive.TankDrive(0, 0);

              autoPhase5Complete = true;
            }
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          mainGyro.Reset();

          autoPhase5Complete = true;
        }
      }

      /*
        Auto Phase 6
        Shoots ball at X RPM.
        Shoot two balls into the goal.
      */
      if (autoPhase5Complete && !autoPhase6Complete) {
        autoPhase6Timer = autoPhase6Timer + 20;

        if (autoPhase6Timer < 1000) {
          double shooterRPM = 318;
          double targetVelocity = limelight.GetFlywheelSpeedFromDistance() * 4096 / 600;

          shooter.shooterMotorL.Set(ControlMode::Velocity, targetVelocity);
          shooter.shooterMotorR.Set(ControlMode::Velocity, targetVelocity);
        } else if (autoPhase6Timer < 3000) {
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(Constant::intakePercentSpeed);
        } else {
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);
          shooter.shooterMotorL.Set(ControlMode::PercentOutput, 0);
          shooter.shooterMotorR.Set(ControlMode::PercentOutput, 0);

          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);
          mainGyro.Reset();

          autoPhase6Complete = true;
        }
      }
      
      /*
        THE FIRST LEFT TURN
        Auto Phase 7
        Turns robot X degrees to the left.
        Turns robot towards second ball.
      */
      if (autoPhase6Complete && !autoPhase7Complete) {
        autoPhase7Timer = autoPhase7Timer + 20;

        if (autoPhase7Timer < 400) return;

        double degrees = -3; // Why do we die

        if (mainGyro.GetAngle() > degrees) {
          driveTrain.differentialDrive.TankDrive(autoTurnSpeed, -autoTurnSpeed);
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          mainGyro.Reset();
          autoPhase7Complete = true;
        }
        
      }
      
      /*
        Auto Phase 8
        Moves robot forward to a set point to intake a ball.
        Moves towards second ball with intake.
      */
      if (autoPhase7Complete && !autoPhase8Complete) {
        double driveSetPoint = 56;
        double speedIncrement = abs(mainGyro.GetAngle()) * Constant::autoStraightPConstant;

        intake.intakeSolenoid.Set(intake.intakeSolenoid.kForward);

        intake.intakeMotor.Set(Constant::intakePercentSpeed);
        intake.indexMotorR.Set(Constant::intakePercentSpeed);
        intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        
        if (positionLeft < driveSetPoint || positionRight < driveSetPoint) {
          if (mainGyro.GetAngle() > Constant::autoStraightDeadzone) { // Too Far Right
            driveTrain.differentialDrive.TankDrive(-autoDriveSpeed, -(autoDriveSpeed + speedIncrement));
          } else if (mainGyro.GetAngle() < -Constant::autoStraightDeadzone) { // Too Far Left
            driveTrain.differentialDrive.TankDrive(-(autoDriveSpeed + speedIncrement), -autoDriveSpeed);
          } else { // Just Right
            driveTrain.differentialDrive.TankDrive(-autoDriveSpeed, -autoDriveSpeed);
          }
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          
          intake.intakeSolenoid.Set(intake.intakeSolenoid.kReverse);
          intake.intakeMotor.Set(0);
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);
          
          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);

          autoPhase8Complete = true;
        }
      }

      /*
        Auto Phase 9
        Runs index motors.
        Indexes ball further.
      */
      if (autoPhase8Complete && !autoPhase9Complete) {
        autoPhase3Timer = autoPhase3Timer + 20;

        if (autoPhase3Timer < 100) {
          intake.indexMotorR.Set(-Constant::intakePercentSpeed);
          intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        } else {
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);

          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);
          mainGyro.Reset();

          autoPhase9Complete = true;
        }
      }

      /*
        Auto Phase 10
        Turns robot X degrees to the left.
        Turns robot towards third ball.
      */

     

      if (autoPhase9Complete && !autoPhase10Complete) {
        //autoPhase7Timer = autoPhase7Timer + 20;

        //if (autoPhase7Timer < 450) return;

        double degrees = -4; // Why do we die: again, drugs

        if (mainGyro.GetAngle() > degrees) {
          driveTrain.differentialDrive.TankDrive(autoTurnSpeed, -autoTurnSpeed);
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          mainGyro.Reset();
          autoPhase10Complete = true;
        }
        
      }

      /*
        Auto Phase 11
        Moves robot forward to a set point to intake a ball.
        Moves towards third ball with intake.
      */

      if (autoPhase10Complete && !autoPhase11Complete) {
        
        double driveSetPoint = 60;
        double speedIncrement = abs(mainGyro.GetAngle()) * Constant::autoStraightPConstant;

        intake.intakeSolenoid.Set(intake.intakeSolenoid.kForward);

        intake.intakeMotor.Set(Constant::intakePercentSpeed);
        intake.indexMotorR.Set(Constant::intakePercentSpeed);
        intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        
        if (positionLeft < driveSetPoint || positionRight < driveSetPoint) {
          if (mainGyro.GetAngle() > Constant::autoStraightDeadzone) { // Too Far Right
            driveTrain.differentialDrive.TankDrive(-autoDriveSpeed, -(autoDriveSpeed + speedIncrement));
          } else if (mainGyro.GetAngle() < -Constant::autoStraightDeadzone) { // Too Far Left
            driveTrain.differentialDrive.TankDrive(-(autoDriveSpeed + speedIncrement), -autoDriveSpeed);
          } else { // Just Right
            driveTrain.differentialDrive.TankDrive(-autoDriveSpeed, -autoDriveSpeed);
          }
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          
          intake.intakeSolenoid.Set(intake.intakeSolenoid.kReverse);
          intake.intakeMotor.Set(0);
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);
          
          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);

          autoPhase11Complete = true;
        }
      }

      /*
        Auto Phase 12
        Runs index motors.
        Indexes ball further.
      */
      if (autoPhase11Complete && !autoPhase12Complete) {
        autoPhase12Timer = autoPhase12Timer + 20;

        if (autoPhase12Timer < 500) {
          intake.intakeMotor.Set(Constant::intakePercentSpeed);
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        } else {
          intake.intakeMotor.Set(0);
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);

          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);
          mainGyro.Reset();

          autoPhase12Complete = true;
        }
      }

      /*
        Auto Phase 13
        Moves robot backwards to set point.
        Moves towards fourth ball.
      */
      if (autoPhase12Complete && !autoPhase13Complete) {
        double driveSetPoint = -40;
        double speedIncrement = abs(mainGyro.GetAngle()) * Constant::autoStraightPConstant;
        
        if (positionLeft > driveSetPoint || positionRight > driveSetPoint) {
          if (mainGyro.GetAngle() < -Constant::autoStraightDeadzone) { // Too Far Right
            driveTrain.differentialDrive.TankDrive(autoDriveSpeed, (autoDriveSpeed + speedIncrement));
          } else if (mainGyro.GetAngle() > Constant::autoStraightDeadzone) { // Too Far Left
            driveTrain.differentialDrive.TankDrive((autoDriveSpeed + speedIncrement), autoDriveSpeed);
          } else { // Just Right
            driveTrain.differentialDrive.TankDrive(autoDriveSpeed, autoDriveSpeed);
          }
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          mainGyro.Reset();

          autoPhase13Complete = true;
        }
      } 

      /*
        Auto Phase 4
        Turns robot X degrees to the right.
        Turns robot towards goal.
      */
      if (autoPhase13Complete && !autoPhase14Complete) {
        double degrees = 100; // Why do we die: drugs

        if (mainGyro.GetAngle() < degrees) {
          driveTrain.differentialDrive.TankDrive(-autoTurnSpeed, autoTurnSpeed);
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          autoPhase14Complete = true;
          mainGyro.Reset();
        }
      }

      if (autoPhase14Complete && !autoPhase15Complete) {
        autoPhase15Timer = autoPhase15Timer + 20;

        if (autoPhase15Timer < 1000) {
          double shooterRPM = 323;
          double targetVelocity = limelight.GetFlywheelSpeedFromDistance() * 4096 / 600;

          shooter.shooterMotorL.Set(ControlMode::Velocity, targetVelocity);
          shooter.shooterMotorR.Set(ControlMode::Velocity, targetVelocity);
        } else if (autoPhase15Timer < 3000) {
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(Constant::intakePercentSpeed);
        } else {
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);
          shooter.shooterMotorL.Set(ControlMode::PercentOutput, 0);
          shooter.shooterMotorR.Set(ControlMode::PercentOutput, 0);

          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);
          mainGyro.Reset();

          autoPhase15Complete = true;
        }
      }

      // Autonomous Complete
      if (autoPhase15Complete) {
        driveTrain.differentialDrive.TankDrive(0, 0);
      }
    }

    void ResetAutonomous() {
      autoPhase0Complete = false;
      autoPhase1Complete = false;
      autoPhase2Complete = false;
      autoPhase3Complete = false;
      autoPhase4Complete = false;
      autoPhase5Complete = false;
      autoPhase6Complete = false;
      autoPhase7Complete = false;
      autoPhase8Complete = false;
      autoPhase9Complete = false;
      autoPhase10Complete = false;
      autoPhase11Complete = false;
      autoPhase12Complete = false;
      autoPhase13Complete = false;
      autoPhase14Complete = false;
      autoPhase15Complete = false;
      autoPhase16Complete = false;

      autoPhase2Timer = 0.0;
      autoPhase3Timer = 0.0;
      autoPhase6Timer = 0.0;
      autoPhase7Timer = 0.0;
      autoPhase9Timer = 0.0;
      autoPhase10Timer = 0.0;
      autoPhase11Timer = 0.0;
      autoPhase12Timer = 0.0;
      autoPhase13Timer = 0.0;
      autoPhase14Timer = 0.0;
      autoPhase15Timer = 0.0;

      driveTrain.leftDriveEncoder.SetPosition(0);
      driveTrain.rightDriveEncoder.SetPosition(0);

      mainGyro.Reset();
    }
};


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
