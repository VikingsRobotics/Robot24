// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/Commands.h>

#include <frc/smartdashboard/SendableChooser.h>

#include "Constants.h"
#include "subsystems/SwerveSubsystem.h"
#include "subsystems/RampSubsystem.h"

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
  frc2::Command* GetAutonomousCommand();

 private:

  void SetSwerveDefaultCommand(frc2::CommandXboxController& control);
  void SetSwerveDefaultCommand(frc2::CommandJoystick& control);

  void GenerateSendable();

  //* Type safe union for driving controller
  std::variant<frc2::CommandXboxController,frc2::CommandJoystick> m_driverController = frc2::CommandJoystick{Operator::kDriverControllerPort};
  frc2::CommandXboxController m_assistController = frc2::CommandXboxController(Operator::kAssistControllerPort);
  //* Swerve Subsystem: Controls robot movement
  SwerveSubsystem m_swerveSubsystem;
  //TODO: ThrowerSubsystem
  RampSubsystem m_rampSubsystem;

  //* Auto routines
  wpi::SmallVector<frc2::CommandPtr,32> m_commands;
  frc::SendableChooser<frc2::Command*> m_chooser;

  //* Config button bindings to controllers for NOW
  void ConfigureBindings();
};
