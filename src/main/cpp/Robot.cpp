#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTable.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <cameraserver/CameraServer.h>
#include <frc/AnalogGyro.h> // Aidan this is for testing, this was not here before.
#include <frc/ADXRS450_Gyro.h>
#include <AHRS.h>

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

    double autoPhase2Timer = 0.0;
    double autoPhase3Timer = 0.0;
    double autoPhase6Timer = 0.0;
    double autoPhase7Timer = 0.0;
    double autoPhase9Timer = 0.0;
    double autoPhase10Timer = 0.0;
    double autoPhase11Timer = 0.0;
    double autoPhase13Timer = 0.0;

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
      m_chooser.AddOption("4-Ball", "4-Ball");
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

      ResetAutonomous();
    };

    void AutonomousPeriodic() override {
      if (m_autoSelected == "2-Ball") { Autonomous1(); }
      else if (m_autoSelected == "3-Ball") { Autonomous2(); }
      else if (m_autoSelected == "4-Ball") { Autonomous4(); }
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

    void TestInit() override {}

    void TestPeriodic() override {};

    void AutonomousStraightTest() {

      double positionLeft = driveTrain.leftDriveEncoder.GetPosition();
      double positionRight = -driveTrain.rightDriveEncoder.GetPosition();

      if (!autoPhase1Complete) {
        mainGyro.Reset();
        autoPhase1Complete = true;
      }
      
      if (autoPhase1Complete && !autoPhase2Complete) {
        double driveSetPoint = 57;

        double deadZone = 0.5;
        double speedIncrement = abs(mainGyro.GetAngle())*0.025;

        if (positionLeft < driveSetPoint || positionRight < driveSetPoint) {
          if (mainGyro.GetAngle() > deadZone) { // Too Far Right
            driveTrain.differentialDrive.TankDrive(-(Constant::autoDriveSpeed + 0.02), -(Constant::autoDriveSpeed + speedIncrement));
          } else if (mainGyro.GetAngle() < -deadZone) { // Too Far Left
            driveTrain.differentialDrive.TankDrive(-(Constant::autoDriveSpeed + speedIncrement), -Constant::autoDriveSpeed);
          } else { // Just Right
            driveTrain.differentialDrive.TankDrive(-Constant::autoDriveSpeed, -Constant::autoDriveSpeed);
          }
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          
          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);

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
          driveTrain.differentialDrive.TankDrive(-Constant::autoTurnSpeed, Constant::autoTurnSpeed);
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          autoPhase2Complete = true;
          mainGyro.Reset();
        }
      }

      if (autoPhase2Complete && !autoPhase3Complete) {
        double degrees = 2.1; // Why do we die

        if (mainGyro.GetAngle() < degrees) {
          driveTrain.differentialDrive.TankDrive(Constant::autoTurnSpeed, -Constant::autoTurnSpeed);
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          autoPhase3Complete = true;
          mainGyro.Reset();
        }
      }

      if (autoPhase3Complete && !autoPhase4Complete) {
        double degrees = -2.1; // Why do we die

        if (mainGyro.GetAngle() > degrees) {
          driveTrain.differentialDrive.TankDrive(-Constant::autoTurnSpeed, Constant::autoTurnSpeed);
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
      double driveSetPoint = 26.5;

      double positionLeft = driveTrain.leftDriveEncoder.GetPosition();
      double positionRight = -driveTrain.rightDriveEncoder.GetPosition();

      if (!autoPhase1Complete) {

        intake.intakeSolenoid.Set(intake.intakeSolenoid.kForward);

        intake.intakeMotor.Set(Constant::intakePercentSpeed);
        intake.indexMotorR.Set(Constant::intakePercentSpeed);
        intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        
        if (positionLeft < driveSetPoint || positionRight < driveSetPoint) {
          if (positionLeft > positionRight) {
            driveTrain.differentialDrive.TankDrive(-0.3, -0.35);
          } else if (positionRight > positionLeft) {
            driveTrain.differentialDrive.TankDrive(-0.35, -0.3);
          } else {
            driveTrain.differentialDrive.TankDrive(-0.3, -0.3);
          }
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);

          intake.intakeSolenoid.Set(intake.intakeSolenoid.kReverse);
          intake.intakeMotor.Set(0);
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);

          autoPhase1Complete = true;
        }
      }

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

          autoPhase2Complete = true;
        }
      }

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

          autoPhase3Complete = true;
        }
      }

      if (autoPhase3Complete && !autoPhase4Complete) {
        double setPoint = 25;

        positionLeft = driveTrain.leftDriveEncoder.GetPosition();
        positionRight = driveTrain.rightDriveEncoder.GetPosition();

        if (positionLeft < setPoint || positionRight < -setPoint) {
          positionLeft = driveTrain.leftDriveEncoder.GetPosition();
          positionRight = driveTrain.rightDriveEncoder.GetPosition();

          driveTrain.differentialDrive.TankDrive(-0.35, 0.35);
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);

          autoPhase4Complete = true;
        }
      }

      if (autoPhase4Complete && !autoPhase5Complete) {
        if (limelight.targetFound == 1) {
            if (limelight.targetOffsetHorizontal < -5) {
                driveTrain.differentialDrive.TankDrive(0.35, -0.35);
            } else if (limelight.targetOffsetHorizontal > 5) {
                driveTrain.differentialDrive.TankDrive(-0.35, 0.35);
            } else {
                driveTrain.differentialDrive.TankDrive(0, 0);

                autoPhase5Complete = true;
            }
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);

          autoPhase5Complete = true;
        }
      }

      if (autoPhase5Complete && !autoPhase6Complete) {
        autoPhase6Timer = autoPhase6Timer + 20;

        if (autoPhase6Timer < 1000) {
          double shooterRPM = 320;
          double targetVelocity = Constant::auto2BallFlywheelSpeed * 4096 / 600;

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

          autoPhase7Complete = true;
        }
      }

      if (autoPhase6Complete) {
        driveTrain.differentialDrive.TankDrive(0, 0);
      }
    };

    void Autonomous2() {

      double positionLeft = driveTrain.leftDriveEncoder.GetPosition();
      double positionRight = -driveTrain.rightDriveEncoder.GetPosition();

      /*
        Auto Phase 1
        Moves robot forward to a set point.
        Moves towards first ball.
      */
      if (!autoPhase1Complete) {
        double driveSetPoint = 24;
        double speedIncrement = abs(mainGyro.GetAngle())*0.025;

        intake.intakeSolenoid.Set(intake.intakeSolenoid.kForward);

        intake.intakeMotor.Set(Constant::intakePercentSpeed);
        intake.indexMotorR.Set(Constant::intakePercentSpeed);
        intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        
        if (positionLeft < driveSetPoint || positionRight < driveSetPoint) {
          if (mainGyro.GetAngle() > Constant::autoStraightDeadzone) { // Too Far Right
            driveTrain.differentialDrive.TankDrive(-Constant::autoDriveSpeed, -(Constant::autoDriveSpeed + speedIncrement));
          } else if (mainGyro.GetAngle() < -Constant::autoStraightDeadzone) { // Too Far Left
            driveTrain.differentialDrive.TankDrive(-(Constant::autoDriveSpeed + speedIncrement), -Constant::autoDriveSpeed);
          } else { // Just Right
            driveTrain.differentialDrive.TankDrive(-Constant::autoDriveSpeed, -Constant::autoDriveSpeed);
          }
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);

          intake.intakeSolenoid.Set(intake.intakeSolenoid.kReverse);
          intake.intakeMotor.Set(0);
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);
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
        double degrees = 179; // Why do we die

        if (mainGyro.GetAngle() < degrees) {
          driveTrain.differentialDrive.TankDrive(-Constant::autoTurnSpeed, Constant::autoTurnSpeed);
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
                driveTrain.differentialDrive.TankDrive(Constant::autoTurnSpeed, -Constant::autoTurnSpeed);
            } else if (limelight.targetOffsetHorizontal > 5) {
                driveTrain.differentialDrive.TankDrive(-Constant::autoTurnSpeed, Constant::autoTurnSpeed);
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
        Auto Phase 7
        Turns robot X degrees to the left.
        Turns robot towards second ball.
      */
      if (autoPhase6Complete && !autoPhase7Complete) {
        double degrees = -65; // Why do we die

        if (mainGyro.GetAngle() > degrees) {
          driveTrain.differentialDrive.TankDrive(Constant::autoTurnSpeed, -Constant::autoTurnSpeed);
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
        double driveSetPoint = 57.1;
        double speedIncrement = abs(mainGyro.GetAngle())*0.015;

        intake.intakeSolenoid.Set(intake.intakeSolenoid.kForward);

        intake.intakeMotor.Set(Constant::intakePercentSpeed);
        intake.indexMotorR.Set(Constant::intakePercentSpeed);
        intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        
        if (positionLeft < driveSetPoint || positionRight < driveSetPoint) {
          if (mainGyro.GetAngle() > Constant::autoStraightDeadzone) { // Too Far Right
            driveTrain.differentialDrive.TankDrive(-Constant::autoDriveSpeed, -(Constant::autoDriveSpeed + speedIncrement));
          } else if (mainGyro.GetAngle() < -Constant::autoStraightDeadzone) { // Too Far Left
            driveTrain.differentialDrive.TankDrive(-(Constant::autoDriveSpeed + speedIncrement), -Constant::autoDriveSpeed);
          } else { // Just Right
            driveTrain.differentialDrive.TankDrive(-Constant::autoDriveSpeed, -Constant::autoDriveSpeed);
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
        Turns robot X degrees to the right.
        Turns robot towards goal.
      */
      if (autoPhase9Complete && !autoPhase10Complete) { //Phase 9
        double degrees = 98; // Why do we die

        if (mainGyro.GetAngle() < degrees) {
          driveTrain.differentialDrive.TankDrive(-Constant::autoTurnSpeed, Constant::autoTurnSpeed);
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          autoPhase10Complete = true;
          mainGyro.Reset();
        }
      }

      /*
        Auto Phase 11
        Shoots ball at X RPM.
        Shoot one ball into the goal.
      */
      if (autoPhase10Complete && !autoPhase11Complete) {
        autoPhase11Timer = autoPhase11Timer + 20;

        if (autoPhase11Timer < 1000) {
          double shooterRPM = 328;
          double targetVelocity = Constant::auto4BallFlywheelSpeed * 4096 / 600;

          shooter.shooterMotorL.Set(ControlMode::Velocity, targetVelocity);
          shooter.shooterMotorR.Set(ControlMode::Velocity, targetVelocity);
        } else if (autoPhase11Timer < 3000) {
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(Constant::intakePercentSpeed);
        } else {
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);
          shooter.shooterMotorL.Set(ControlMode::PercentOutput, 0);
          shooter.shooterMotorR.Set(ControlMode::PercentOutput, 0);

          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);

          autoPhase11Complete = true;
        }
      }

      // Autonomous Complete
      if (autoPhase11Complete) {
        driveTrain.differentialDrive.TankDrive(0, 0);
      }
    }

    void Autonomous3() {

      double positionLeft = driveTrain.leftDriveEncoder.GetPosition();
      double positionRight = -driveTrain.rightDriveEncoder.GetPosition();
      
      if (!autoPhase1Complete) {
        double driveSetPoint = 35;

        intake.intakeSolenoid.Set(intake.intakeSolenoid.kForward);

        intake.intakeMotor.Set(Constant::intakePercentSpeed);
        intake.indexMotorR.Set(Constant::intakePercentSpeed);
        intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        
        if (positionLeft < driveSetPoint || positionRight < driveSetPoint) {
          if (positionLeft > positionRight) {
            driveTrain.differentialDrive.TankDrive(-Constant::autoDriveSpeed, -Constant::autoDriveSpeed - Constant::autoStraight);
          } else if (positionRight > positionLeft) {
            driveTrain.differentialDrive.TankDrive(-Constant::autoDriveSpeed - Constant::autoStraight, -Constant::autoDriveSpeed);
          } else {
            driveTrain.differentialDrive.TankDrive(-0.3, -0.3);
          }
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);

          intake.intakeSolenoid.Set(intake.intakeSolenoid.kReverse);
          intake.intakeMotor.Set(0);
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);

          autoPhase1Complete = true;
        }
      } 

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

          autoPhase2Complete = true;
        }
      }

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

          autoPhase3Complete = true;
        }
      }

      if (autoPhase3Complete && !autoPhase4Complete) {
        double setPoint = 21;

        positionLeft = driveTrain.leftDriveEncoder.GetPosition();
        positionRight = driveTrain.rightDriveEncoder.GetPosition();

        if (positionLeft < setPoint || positionRight < -setPoint) {
          positionLeft = driveTrain.leftDriveEncoder.GetPosition();
          positionRight = driveTrain.rightDriveEncoder.GetPosition();

          driveTrain.differentialDrive.TankDrive(-Constant::autoTurnSpeed, Constant::autoTurnSpeed);
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);

          autoPhase4Complete = true;
        }
      }

      if (autoPhase4Complete && !autoPhase5Complete) {
        if (limelight.targetFound == 1) {
            if (limelight.targetOffsetHorizontal < -5) {
                driveTrain.differentialDrive.TankDrive(Constant::autoTurnSpeed, -Constant::autoTurnSpeed);
            } else if (limelight.targetOffsetHorizontal > 5) {
                driveTrain.differentialDrive.TankDrive(-Constant::autoTurnSpeed, Constant::autoTurnSpeed);
            } else {
              driveTrain.differentialDrive.TankDrive(0, 0);

              autoPhase5Complete = true;
            }
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);

          autoPhase5Complete = true;
        }
      }

      
      if (autoPhase5Complete && !autoPhase6Complete) {
        autoPhase6Timer = autoPhase6Timer + 20;

        if (autoPhase6Timer < 1000) {
          double shooterRPM = 323;
          double targetVelocity = Constant::auto4BallFlywheelSpeed * 4096 / 600;

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

          autoPhase6Complete = true;
        }
      } // After Both Balls are shot
      
      // comment block start
      if (autoPhase6Complete && !autoPhase7Complete) {
        double setPoint = 21;

        positionLeft = driveTrain.leftDriveEncoder.GetPosition();
        positionRight = driveTrain.rightDriveEncoder.GetPosition();

        if (positionLeft > setPoint || positionRight > -setPoint) {
          positionLeft = driveTrain.leftDriveEncoder.GetPosition();
          positionRight = driveTrain.rightDriveEncoder.GetPosition();

          driveTrain.differentialDrive.TankDrive(Constant::autoTurnSpeed, -Constant::autoTurnSpeed);
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);

          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);

          autoPhase7Complete = true;
        }
        
      }
      
      if (autoPhase7Complete && !autoPhase8Complete) {
        double driveSetPoint = 85.5;

        if (positionLeft < driveSetPoint || positionRight < driveSetPoint) {

          if (positionLeft > 70 || positionRight > 70) {
            intake.intakeSolenoid.Set(intake.intakeSolenoid.kForward);

            intake.intakeMotor.Set(Constant::intakePercentSpeed);
            intake.indexMotorR.Set(Constant::intakePercentSpeed);
            intake.indexMotorL.Set(-Constant::intakePercentSpeed);
          }

          if (positionLeft > positionRight) {
            driveTrain.differentialDrive.TankDrive(-Constant::autoDriveSpeed, -Constant::autoDriveSpeed - Constant::autoStraight);
          } else if (positionRight > positionLeft) {
            driveTrain.differentialDrive.TankDrive(-Constant::autoDriveSpeed - Constant::autoStraight, -Constant::autoDriveSpeed);
          } else {
            driveTrain.differentialDrive.TankDrive(-Constant::autoDriveSpeed, -Constant::autoDriveSpeed);
          }
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);

          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);

          autoPhase8Complete = true;
        }
      }

      if (autoPhase8Complete && !autoPhase9Complete) {
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

          intake.intakeMotor.Set(0);
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);

          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);

          autoPhase9Complete = true;
        }
      }

      if (autoPhase9Complete && !autoPhase10Complete) {
        double driveSetPoint = -85.5;

        if (positionLeft > driveSetPoint || positionRight > driveSetPoint) {

          if (positionLeft < positionRight) {
            driveTrain.differentialDrive.TankDrive(Constant::autoDriveSpeed, Constant::autoDriveSpeed + Constant::autoStraight);
          } else if (positionRight < positionLeft) {
            driveTrain.differentialDrive.TankDrive(Constant::autoDriveSpeed + Constant::autoStraight, Constant::autoDriveSpeed);
          } else {
            driveTrain.differentialDrive.TankDrive(Constant::autoDriveSpeed, Constant::autoDriveSpeed);
          }
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);

          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);

          autoPhase10Complete = true;
        }
      }

      if (autoPhase10Complete && !autoPhase11Complete) {
        double setPoint = -22;

        positionLeft = driveTrain.leftDriveEncoder.GetPosition();
        positionRight = driveTrain.rightDriveEncoder.GetPosition();

        if (positionLeft < setPoint || positionRight < -setPoint) {
          positionLeft = driveTrain.leftDriveEncoder.GetPosition();
          positionRight = driveTrain.rightDriveEncoder.GetPosition();

          driveTrain.differentialDrive.TankDrive(-Constant::autoTurnSpeed, Constant::autoTurnSpeed);
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);

          autoPhase11Complete = true;
        }
      }

      if (autoPhase11Complete && !autoPhase12Complete) {

        autoPhase12Complete = true;

        
        if (limelight.targetFound == 1) {
            if (limelight.targetOffsetHorizontal < -5) {
                driveTrain.differentialDrive.TankDrive(Constant::autoTurnSpeed, -Constant::autoTurnSpeed);
            } else if (limelight.targetOffsetHorizontal > 5) {
                driveTrain.differentialDrive.TankDrive(-Constant::autoTurnSpeed, Constant::autoTurnSpeed);
            } else {
              driveTrain.differentialDrive.TankDrive(0, 0);

              autoPhase12Complete = true;
            }
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);

          autoPhase12Complete = true;
        }
        
      }

      if (autoPhase12Complete && !autoPhase13Complete) {
        autoPhase13Timer = autoPhase13Timer + 20;

        if (autoPhase13Timer < 1000) {
          double shooterRPM = 320;
          double targetVelocity = shooterRPM * 4096 / 600;

          shooter.shooterMotorL.Set(ControlMode::Velocity, targetVelocity);
          shooter.shooterMotorR.Set(ControlMode::Velocity, targetVelocity);
        } else if (autoPhase6Timer < 5000) {
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(Constant::intakePercentSpeed);
        } else {
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);
          shooter.shooterMotorL.Set(ControlMode::PercentOutput, 0);
          shooter.shooterMotorR.Set(ControlMode::PercentOutput, 0);

          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);

          autoPhase13Complete = true;
        }
      }
      //comment block end

      if (autoPhase13Complete) { // This is dangerous
        driveTrain.differentialDrive.TankDrive(0, 0);
      }
    }

    void Autonomous4() {

      double positionLeft = driveTrain.leftDriveEncoder.GetPosition();
      double positionRight = -driveTrain.rightDriveEncoder.GetPosition();

      /*
        Auto Phase 1
        Moves robot forward to a set point.
        Moves towards first ball.
      */
      if (!autoPhase1Complete) {
        double driveSetPoint = 24;
        double speedIncrement = abs(mainGyro.GetAngle())*0.025;

        intake.intakeSolenoid.Set(intake.intakeSolenoid.kForward);

        intake.intakeMotor.Set(Constant::intakePercentSpeed);
        intake.indexMotorR.Set(Constant::intakePercentSpeed);
        intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        
        if (positionLeft < driveSetPoint || positionRight < driveSetPoint) {
          if (mainGyro.GetAngle() > Constant::autoStraightDeadzone) { // Too Far Right
            driveTrain.differentialDrive.TankDrive(-Constant::autoDriveSpeed, -(Constant::autoDriveSpeed + speedIncrement));
          } else if (mainGyro.GetAngle() < -Constant::autoStraightDeadzone) { // Too Far Left
            driveTrain.differentialDrive.TankDrive(-(Constant::autoDriveSpeed + speedIncrement), -Constant::autoDriveSpeed);
          } else { // Just Right
            driveTrain.differentialDrive.TankDrive(-Constant::autoDriveSpeed, -Constant::autoDriveSpeed);
          }
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);

          intake.intakeSolenoid.Set(intake.intakeSolenoid.kReverse);
          intake.intakeMotor.Set(0);
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);
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
        double degrees = 179; // Why do we die

        if (mainGyro.GetAngle() < degrees) {
          driveTrain.differentialDrive.TankDrive(-Constant::autoTurnSpeed, Constant::autoTurnSpeed);
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
                driveTrain.differentialDrive.TankDrive(Constant::autoTurnSpeed, -Constant::autoTurnSpeed);
            } else if (limelight.targetOffsetHorizontal > 5) {
                driveTrain.differentialDrive.TankDrive(-Constant::autoTurnSpeed, Constant::autoTurnSpeed);
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
        Auto Phase 7
        Turns robot X degrees to the left.
        Turns robot towards second ball.
      */
      if (autoPhase6Complete && !autoPhase7Complete) {
        double degrees = -65; // Why do we die

        if (mainGyro.GetAngle() > degrees) {
          driveTrain.differentialDrive.TankDrive(Constant::autoTurnSpeed, -Constant::autoTurnSpeed);
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
        double driveSetPoint = 57.1;
        double speedIncrement = abs(mainGyro.GetAngle())*0.015;

        intake.intakeSolenoid.Set(intake.intakeSolenoid.kForward);

        intake.intakeMotor.Set(Constant::intakePercentSpeed);
        intake.indexMotorR.Set(Constant::intakePercentSpeed);
        intake.indexMotorL.Set(-Constant::intakePercentSpeed);
        
        if (positionLeft < driveSetPoint || positionRight < driveSetPoint) {
          if (mainGyro.GetAngle() > Constant::autoStraightDeadzone) { // Too Far Right
            driveTrain.differentialDrive.TankDrive(-Constant::autoDriveSpeed, -(Constant::autoDriveSpeed + speedIncrement));
          } else if (mainGyro.GetAngle() < -Constant::autoStraightDeadzone) { // Too Far Left
            driveTrain.differentialDrive.TankDrive(-(Constant::autoDriveSpeed + speedIncrement), -Constant::autoDriveSpeed);
          } else { // Just Right
            driveTrain.differentialDrive.TankDrive(-Constant::autoDriveSpeed, -Constant::autoDriveSpeed);
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
        Turns robot X degrees to the right.
        Turns robot towards goal.
      */
      if (autoPhase9Complete && !autoPhase10Complete) { //Phase 9
        double degrees = 98; // Why do we die

        if (mainGyro.GetAngle() < degrees) {
          driveTrain.differentialDrive.TankDrive(-Constant::autoTurnSpeed, Constant::autoTurnSpeed);
        } else {
          driveTrain.differentialDrive.TankDrive(0, 0);
          autoPhase10Complete = true;
          mainGyro.Reset();
        }
      }

      /*
        Auto Phase 11
        Shoots ball at X RPM.
        Shoot one ball into the goal.
      */
      if (autoPhase10Complete && !autoPhase11Complete) {
        autoPhase11Timer = autoPhase11Timer + 20;

        if (autoPhase11Timer < 1000) {
          double shooterRPM = 328;
          double targetVelocity = Constant::auto4BallFlywheelSpeed * 4096 / 600;

          shooter.shooterMotorL.Set(ControlMode::Velocity, targetVelocity);
          shooter.shooterMotorR.Set(ControlMode::Velocity, targetVelocity);
        } else if (autoPhase11Timer < 3000) {
          intake.indexMotorR.Set(Constant::intakePercentSpeed);
          intake.indexMotorL.Set(Constant::intakePercentSpeed);
        } else {
          intake.indexMotorR.Set(0);
          intake.indexMotorL.Set(0);
          shooter.shooterMotorL.Set(ControlMode::PercentOutput, 0);
          shooter.shooterMotorR.Set(ControlMode::PercentOutput, 0);

          driveTrain.leftDriveEncoder.SetPosition(0);
          driveTrain.rightDriveEncoder.SetPosition(0);

          autoPhase11Complete = true;
        }
      }

      // Autonomous Complete
      if (autoPhase11Complete) {
        driveTrain.differentialDrive.TankDrive(0, 0);
      }
    }

    void ResetAutonomous() {
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

      autoPhase2Timer = 0.0;
      autoPhase3Timer = 0.0;
      autoPhase6Timer = 0.0;
      autoPhase7Timer = 0.0;
      autoPhase9Timer = 0.0;
      autoPhase10Timer = 0.0;
      autoPhase11Timer = 0.0;
      autoPhase13Timer = 0.0;

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
