// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
 public:
  //* This function is called once when the robot is being initialized.
  void RobotInit() override;
  /**
   * This function is called every 20 ms, no matter the mode. Use
   * this for items like diagnostics that you want to run during disabled,
   * autonomous, teleoperated and test.
   *
   * This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  void RobotPeriodic() override;
  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  void DisabledInit() override;
  //* This function is called periodically during disabled control.
  void DisabledPeriodic() override;
  //* This function is called once when the robot is starting autonomous.
  void AutonomousInit() override;
  //* This function is called periodically during autonomous control.
  void AutonomousPeriodic() override;
  //* This function is called once during operator control.
  void TeleopInit() override;
  //* This function is called periodically during operator control.
  void TeleopPeriodic() override;
  //* This function is called periodically during test mode.
  void TestPeriodic() override;
  //* This function is called once when the robot is first started up.
  void SimulationInit() override;
  //* This function is called periodically whilst in simulation.
  void SimulationPeriodic() override;

 private:
  //* A command that is explicitly scheduled during auto and cancelled during teleop
  std::optional<frc2::CommandPtr> m_autonomousCommand;
  //* All of the robot interal storage
  RobotContainer m_container;
};
