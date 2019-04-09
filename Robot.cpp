/* Last Updated April 9, 2019 
* By Lauren Williams
* Description- 
*/

#include <frc/Joystick.h> // Joystick Library
#include <frc/TimedRobot.h> // Timed Robot Library
#include "ctre/Phoenix.h" // CTRE Library
#include <iostream> // IO Stream Library 
#include <string> // String Library 
#include "frc/WPILib.h" // WPI Library
#include <math.h> // Math Library 
#include <frc/drive/DifferentialDrive.h> // Differential Drive Library
#include <rev/CANSparkMax.h> // Library for SPARK Max Motor Contorllers

using namespace frc;

class Robot: public TimedRobot 
{
	frc::DigitalInput *liftLowerLimit = new DigitalInput(0); // Lower Limit Switch DOI port 0
	frc::DigitalInput *liftUpperLimit = new DigitalInput(1); // Upper Limit Switch DOI port 1

public:
	
	// Motor Controllers
  // Victor Motor Controllers
	WPI_VictorSPX * Lift = new WPI_VictorSPX(8); // Victor SPX Lift motor Victor #8
	WPI_VictorSPX * fourBar = new WPI_VictorSPX(5); // Victor SPX Four Bar motor Victor #5
	WPI_VictorSPX * intakeRoll = new WPI_VictorSPX(7); // Victor SPX Cargo Intake motor Victor #7
	WPI_VictorSPX * climberTrack = new WPI_VictorSPX(10); // Victor SPX Climber Track Victor #10

  // REV SPARK Max Motor Controllers
	static const int leftLeadDeviceID = 1, leftFollowDeviceID = 3, rightLeadDeviceID = 4, rightFollowDeviceID = 2; // Sets the device ID for each contoller
  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless}; // Sets the device ID equal to the label m_leftLeadMotor and sets the motor type to brushless
  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless}; // Sets the device ID equal to the label m_rightLeadMotor and sets the motor type to brushless
  rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless}; // Sets the device ID equal to the label m_leftFollowerMotor and sets the motor type to brushless
  rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless}; // Sets the device ID equal to the label m_rightFollowerMotor and sets the motor type to brushless

	// Pneumatics  
	Compressor *c = new Compressor(0); // Compressor CAN ID #0
	frc::DoubleSolenoid Jacks {0,0,1}; // Sets the Double Solenoid to PCM ports 0,1 and tells the robot that the PCM is CAN ID #0
	frc::DoubleSolenoid Wheelie {0,6,7}; // Sets the Double Solenoid to PCM ports 6,7 and tells the robot that the PCM is CAN ID #0

	// Robot Drive
  frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor}; // Sets up the Differential Drive for the robot drive system

	// Joystick
	Joystick * drive = new Joystick(1); // Creates the Drive joystick variable and sets it to USB port 1
	Joystick * auxiliary = new Joystick(0); // Creates the Auxiliary joystick variable and sets it to USB port 0

  // Jack Subroutine
  void Jack()
  {
    if(drive->GetRawButton(5))
    {
      Jacks.Set(frc::DoubleSolenoid::Value::kForward);
    }
    if(drive->GetRawButton(6))
    {
      Jacks.Set(frc::DoubleSolenoid::kReverse);
    }
  }

  // Climber Track Subroutine 
  void ClimberTrack()
  {
    if(auxiliary->GetRawButton(7))
    {
      climberTrack->Set(8);
    }
    else if(auxiliary->GetRawButton(8))
    {
      climberTrack->Set(-8);
    }
    else
    {
      climberTrack->Set(0);
    }
  }

  // Wheelie Bar Subroutine 
  void WheelieBar()
  {
    if(auxiliary->GetRawButton(5))
    {
      Wheelie.Set(frc::DoubleSolenoid::kForward);
    }
    if(auxiliary->GetRawButton(6))
    {
      Wheelie.Set(frc::DoubleSolenoid::kReverse);
    }
  }

  // Four Bar Subroutine 
  void FourBar()
  {
    if(auxiliary->GetRawButton(1))
    {
      fourBar->Set(8);
    }
    else if(auxiliary->GetRawButton(2))
    {
      fourBar->Set(-1);
    }
    else 
    {
      fourBar->Set(0);
    }
  }

  // Intake Roll Subroutine 
  void IntakeRoll()
  {
    if(auxiliary->GetRawButton(7))
    {
      intakeRoll->Set(8);
    }
    else if(auxiliary->GetRawButton(8))
    {
      intakeRoll->Set(-8);
    }
    else
    {
      intakeRoll->Set(0);
    }
  }

  // Lift Subroutine 
  void Elevator()
  {
    Lift->Set(-auxiliary->GetY());
  }

  // Robot Drive Subroutine
  void RobotDriveDiff()
  {
    m_robotDrive.ArcadeDrive(drive->GetY(), drive->GetZ());
  }

  // Lift Lower Limit Subroutine 
  void LowerLimit()
  {
    while(liftLowerLimit->Get())
    {
      Elevator();
      FourBar();
      ClimberTrack();
      Jack();
      IntakeRoll();
      WheelieBar();
      RobotDriveDiff();
      c->SetClosedLoopControl(true);
    }
  }

  // Lift Upper Limit 
  void UpperLimit()
  {
    while(liftUpperLimit->Get())
    {
      Elevator();
      FourBar();
      ClimberTrack();
      Jack();
      IntakeRoll();
      WheelieBar();
      RobotDriveDiff();
      c->SetClosedLoopControl(true);
    }
  }

  // Opperator Control Subroutine
  void OpperatorControl()
  {
    LowerLimit();
    UpperLimit();
    FourBar();
    IntakeRoll();
    ClimberTrack();
    Jack();
    WheelieBar();
    Lift->Set(0);
    c->SetClosedLoopControl(true);
  }

  // Teleop Periodic 
  void TeleopPeriodic() 
  {
    OpperatorControl();
  }

  // Autonomous Peridoic 
  void AutonomousPeriodic()
  {
    OpperatorControl();
  }

  // Robot Init 
  void RobotInit()
  {
    CameraServer::GetInstance()->StartAutomaticCapture();
		CameraServer::GetInstance()->StartAutomaticCapture();

    c->SetClosedLoopControl(true);

    m_leftLeadMotor.RestoreFactoryDefaults();
    m_rightLeadMotor.RestoreFactoryDefaults();
    m_leftFollowMotor.RestoreFactoryDefaults();
    m_rightFollowMotor.RestoreFactoryDefaults();

    m_leftLeadMotor.SetInverted(true);
    m_leftFollowMotor.SetInverted(false);
    m_rightLeadMotor.SetInverted(true);
    m_rightFollowMotor.SetInverted(true);
    
    
    m_leftFollowMotor.Follow(m_leftLeadMotor);
    m_rightFollowMotor.Follow(m_rightLeadMotor);
  }
private:
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif