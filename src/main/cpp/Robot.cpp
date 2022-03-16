#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTable.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <cameraserver/CameraServer.h>

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
    double autoPhase13Timer = 0.0;

    frc::SendableChooser<std::string> m_chooser;
    const std::string kAutoNameDefault = "Default";
    const std::string kAutoNameCustom = "My Auto";
    std::string m_autoSelected;

  public:

    void RobotInit() override {
      frc::CameraServer::StartAutomaticCapture();

      driveTrain.Init(driveTrain);
      shooter.Init(shooter);
      intake.Init(intake);
      climb.Init(climb);

      m_chooser.SetDefaultOption("2-Ball", "2-Ball");
      m_chooser.AddOption("4-Ball", "4-Ball");
      frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
      
      frc::SmartDashboard::PutNumber("Shooter RPM", 320);
    };

    void RobotPeriodic() override {
      limelight.UpdateLimelightValues();
    };

    void AutonomousInit() override {
      limelight.EnableLEDs();

      m_autoSelected = m_chooser.GetSelected();

      ResetAutonomous();
    };

    void AutonomousPeriodic() override {
      if (m_autoSelected == "2-Ball") {
        Autonomous1();
      } else {
        Autonomous2();
      }
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
      }
      
      /*
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
      */

      if (autoPhase6Complete) {
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
      autoPhase13Timer = 0.0;

      driveTrain.leftDriveEncoder.SetPosition(0);
      driveTrain.rightDriveEncoder.SetPosition(0);
    }
};


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
