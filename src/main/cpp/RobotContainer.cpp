// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <units/time.h>

#include "commands/SwerveDriveCommand.h"
#include "commands/SwerveTesterCommand.h"

template<class... Ts>
struct overloaded : Ts... { using Ts::operator()...; };
// explicit deduction guide (not needed as of C++20)
template<class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  // Swerve Subsystem defaults back to user controlled controller command

  //Currently m_driverController is monostate
  try
  {
    //Try to assign xbox controller
    m_driverController = frc2::CommandXboxController{Operator::kDriverControllerPort};
  }
  catch(const std::exception& e)
  {
    try
    {
      // Failed to assign xbox controller so try to get joystick
      m_driverController = frc2::CommandJoystick{Operator::kDriverControllerPort};
    }
    catch(const std::exception& ex)
    {
      // Failed at both, just kill your program
      throw std::runtime_error("Controller input not accepted");
    }
  }
  std::visit(overloaded{
    // Assign default command for joystick
    [this](frc2::CommandJoystick& control)
    {
      m_swerveSubsystem.SetDefaultCommand(SwerveDriveCommand(&m_swerveSubsystem,
        [&]{return control.GetX();},
        [&]{return control.GetY();},
        [&]{return control.GetZ();},
        [&]{return control.GetRawButton(2);},
        [&]{return control.GetRawButton(1);}));
    },
    // Assign default command for xbox
    [this](frc2::CommandXboxController& control)
    {
      m_swerveSubsystem.SetDefaultCommand(SwerveDriveCommand(&m_swerveSubsystem,
        [&]{return control.GetRightY();},
        [&]{return control.GetRightX();},
        [&]{return control.GetLeftX();},
        [&]{return control.X().Get();},
        [&]{return control.GetLeftTriggerAxis() > 0.7;}));
    },
    // How is this possible, put here as last case to stop UB and not because it is required by visit
    [this](std::monostate& YOU_ARE_TRYING_TO_ACCESS_A_INVALID_STATE_WHICH_IS_NOT_ALLOWED)
    { throw std::runtime_error("Failed to assign default command"); }
  },m_driverController);    
  // Put the command onto the dashboard so it can be scheduled if something take Swerve Subsystem
  frc::SmartDashboard::PutData("Drive Command", m_swerveSubsystem.GetDefaultCommand());
  // Sets the name of SwerveSubsystem, called "Swerve System"
  m_swerveSubsystem.SetName("Swerve System");
  
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  
}

std::optional<frc2::CommandPtr> RobotContainer::GetAutonomousCommand() {
  // Currently returns a empty CommandPtr
  return {}; 
}
