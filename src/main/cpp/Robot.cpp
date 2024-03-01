// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <pathplanner/lib/pathfinding/Pathfinding.h>
#include <pathplanner/lib/pathfinding/LocalADStar.h>

void Robot::RobotInit() {
  std::unique_ptr<pathplanner::Pathfinder> pathFinder{std::make_unique<pathplanner::LocalADStar>()};
  pathplanner::Pathfinding::setPathfinder(std::move(pathFinder));
  frc::SmartDashboard::PutData(&frc2::CommandScheduler::GetInstance());
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() {
  m_container.GetSwerve().ZeroHeading();
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand.value()->Schedule();
  }
}

void Robot::AutonomousPeriodic() {
  if(!m_autonomousCommand.value()->IsScheduled())
  {
    m_autonomousCommand = m_container.GetBrakeCommand();
    m_autonomousCommand.value()->Schedule();
  }
}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand) {
    m_autonomousCommand.value()->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
