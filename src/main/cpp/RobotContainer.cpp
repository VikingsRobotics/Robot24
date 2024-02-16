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

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  // Swerve Subsystem defaults back to user controlled controller command
  m_swerveSubsystem.SetDefaultCommand(SwerveDriveCommand(&m_swerveSubsystem,
    [&]()->double{return m_driverController.GetRightY();},
    [&]()->double{return m_driverController.GetRightX();},
    [&]()->double{return m_driverController.GetLeftX();}));
    
  // Put the command onto the dashboard so it can be scheduled if something take Swerve Subsystem
  frc::SmartDashboard::PutData("Drive Command", m_swerveSubsystem.GetDefaultCommand());
  // Sets the name of SwerveSubsystem, called "Swerve System"
  m_swerveSubsystem.SetName("Swerve System");
  
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Triggers SwerveTesterCommand to run
  m_driverController.A().OnTrue(SwerveTesterCommand(&m_swerveSubsystem, SwerveTesterCommand::MaxToZero, 20_s).ToPtr());
}

std::optional<frc2::CommandPtr> RobotContainer::GetAutonomousCommand() {
  // Currently returns a empty CommandPtr
  return {}; 
}
