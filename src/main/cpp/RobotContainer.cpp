// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include <units/time.h>

#include <cmath>

#include <frc2/command/Commands.h>
#include "commands/SwerveDriveCommand.h"
#include "commands/SwerveTesterCommand.h"

//Stolen code from cppreference for std::visit
template<class... Ts>
struct overloaded : Ts... { using Ts::operator()...; };
// explicit deduction guide (not needed as of C++20)
template<class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  // Swerve Subsystem defaults back to user controlled controller command
  
  m_chooser.SetDefaultOption("Default Auto",m_defaultAuto.get());
  m_chooser.AddOption("Move-Only Auto",m_moveAuto.get());
  m_chooser.AddOption("Throw Auto",m_throwAuto.get());
  m_chooser.AddOption("Collect Auto",m_collectAuto.get());
  frc::SmartDashboard::PutData(&m_chooser);

  // Get the type of controller
  frc::GenericHID::HIDType type;
  {
    frc::GenericHID temp{Operator::kDriverControllerPort};
    type = temp.GetType();
  }
  //Currently m_driverController is monostate
  if(type == frc::GenericHID::HIDType::kHIDJoystick) { m_driverController = frc2::CommandJoystick{Operator::kDriverControllerPort}; }
  else { m_driverController = frc2::CommandXboxController{Operator::kDriverControllerPort}; }

  // Visit the the current m_driverController for default command
  std::visit(overloaded{
    // Assign default command for joystick
    [this](frc2::CommandJoystick& control)
    {
      m_swerveSubsystem.SetDefaultCommand(SwerveDriveCommand(&m_swerveSubsystem,
        [&]{return -control.GetThrottle() < Operator::Drive::kPrecisionThrottleThreshold ? 
          Operator::Drive::kPercentDrivePrecision * control.GetX() : 
          std::lerp(Operator::Drive::kPercentDriveLow,1,(-control.GetThrottle() - Operator::Drive::kPrecisionThrottleThreshold) / (1 - Operator::Drive::kPrecisionThrottleThreshold)) * control.GetX();},
        [&]{return -control.GetThrottle() < Operator::Drive::kPrecisionThrottleThreshold ? 
          Operator::Drive::kPercentDrivePrecision * control.GetY() : 
          std::lerp(Operator::Drive::kPercentDriveLow,1,(-control.GetThrottle() - Operator::Drive::kPrecisionThrottleThreshold) / (1 - Operator::Drive::kPrecisionThrottleThreshold)) * control.GetY();},
        [&]{return -control.GetThrottle() < Operator::Drive::kPrecisionThrottleThreshold ? 
          Operator::Drive::kPercentDrivePrecision * control.GetTwist() : 
          std::lerp(Operator::Drive::kPercentDriveLow,1,(-control.GetThrottle() - Operator::Drive::kPrecisionThrottleThreshold) / (1 - Operator::Drive::kPrecisionThrottleThreshold)) * control.GetTwist();},
        [&]{return control.GetRawButton(2);},
        [&]{return control.GetRawButton(1);}));
    },
    // Assign default command for xbox
    [this](frc2::CommandXboxController& control)
    {
      m_swerveSubsystem.SetDefaultCommand(SwerveDriveCommand(&m_swerveSubsystem,
        [&]{return control.RightBumper().Get() ? 
          Operator::Drive::kPercentDrivePrecision * control.GetRightY() :
          std::lerp(Operator::Drive::kPercentDriveLow,1,control.GetRightTriggerAxis()) * control.GetRightY();},
        [&]{return control.RightBumper().Get() ? 
          Operator::Drive::kPercentDrivePrecision * control.GetRightX() : 
          std::lerp(Operator::Drive::kPercentDriveLow,1,control.GetRightTriggerAxis()) * control.GetRightX();}, 
        [&]{return control.RightBumper().Get() ? 
          Operator::Drive::kPercentDrivePrecision * control.GetLeftX() : 
          std::lerp(Operator::Drive::kPercentDriveLow,1,control.GetRightTriggerAxis()) * control.GetLeftX();}, 
        [&]{return control.X().Get();},
        [&]{return control.GetLeftTriggerAxis() > 0.7;}));
    },
    // How is this possible, put here as last case to stop UB and not because it is required by visit
    [this](std::monostate& YOU_ARE_TRYING_TO_ACCESS_A_INVALID_STATE_WHICH_IS_NOT_ALLOWED)
    { throw std::runtime_error("Failed to assign default command"); }
  },m_driverController);    

  // Put the command onto the dashboard so it can be scheduled if something take Swerve Subsystem
  frc::SmartDashboard::PutData("Drive Command", m_swerveSubsystem.GetDefaultCommand());
  
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Currently returns a empty CommandPtr


  return m_chooser.GetSelected(); 
}
