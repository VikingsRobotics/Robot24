// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandJoystick.h>

#include "Constants.h"
#include "subsystems/SwerveSubsystem.h"

#include <optional>
#include <variant>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the robot periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  /**
   * Constructs all of the storage for the robot
  */
  RobotContainer();
  //* @return CommandPtr for auto, TODO
  std::optional<frc2::CommandPtr> GetAutonomousCommand();

 private:
  //* Type safe union for driving controller
  std::variant<std::monostate,frc2::CommandXboxController,frc2::CommandJoystick> m_driverController{std::monostate{}};

  //* Swerve Subsystem: Controls robot movement
  SwerveSubsystem m_swerveSubsystem;
  //*
  //TODO: GrabberSubsystem
  //*
  //TODO: ThrowerSubsystem

  //* Config button bindings to controllers for NOW
  void ConfigureBindings();
};
