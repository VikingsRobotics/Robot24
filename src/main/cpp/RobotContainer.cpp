// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/SwerveDriveCommand.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  m_swerveSubsystem.SetDefaultCommand(SwerveDriveCommand(&m_swerveSubsystem,
    [&]()->double{return m_driverController.GetRightX();},
    [&]()->double{return m_driverController.GetRightY();},
    [&]()->double{return m_driverController.GetLeftX();}));
  m_swerveSubsystem.SetName("Swerve System");
  
  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous

}
