// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <units/time.h>
#include <units/angular_velocity.h>

#include <cmath>

#include <frc2/command/Commands.h>
#include <frc2/command/ConditionalCommand.h>
#include "commands/SwerveDriveCommand.h"
#include "commands/SwerveTesterCommand.h"
#include "commands/RampGatherCommand.h"
#include "commands/RampLaunchCommand.h"
#include "commands/RampLiftCommand.h"
#include "commands/RampDropCommand.h"

#include <filesystem>
#include <frc/Filesystem.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>

RobotContainer::RobotContainer() 
{
  // Initialize all of your commands and subsystems here
#if !defined(REMOVE_SWERVE) && !defined(REMOVE_AUTO)
  #ifndef REMOVE_RAMP
  pathplanner::NamedCommands::registerCommand("RampLaunchDownCommand",RampLaunchCommand{&m_rampSubsystem,false,false}.ToPtr());
  pathplanner::NamedCommands::registerCommand("RampLaunchUpCommand",RampLaunchCommand{&m_rampSubsystem,false,true}.ToPtr());
  pathplanner::NamedCommands::registerCommand("RampLaunchCommand",RampLaunchCommand{&m_rampSubsystem,true}.ToPtr());
  pathplanner::NamedCommands::registerCommand("RampGatherCommand",RampGatherCommand{&m_rampSubsystem}.ToPtr());
  #endif
  GenerateSendable();  
  frc::SmartDashboard::PutData(&m_chooser);
#endif
#ifndef REMOVE_SWERVE
  // Get the type of controller
  //if(std::get<frc2::CommandJoystick>(m_driverController).GetType() == frc::GenericHID::HIDType::kHIDJoystick) 
  { 
    m_driverController = frc2::CommandJoystick{Operator::kDriverControllerPort};
    SetSwerveDefaultCommandJoy(std::get<frc2::CommandJoystick>(m_driverController));
  }
  //else 
  //{ 
  //  m_driverController = frc2::CommandXboxController{Operator::kDriverControllerPort}; 
  //  SetSwerveDefaultCommandXbox(std::get<frc2::CommandXboxController>(m_driverController));
  //}

  // Put the command onto the dashboard so it can be scheduled if something take Swerve Subsystem
  frc::SmartDashboard::PutData("Drive Command", m_swerveSubsystem.GetDefaultCommand());
#endif
  
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
#ifndef REMOVE_RAMP
  #ifndef REMOVE_SOLENOID
  frc2::CommandPtr liftUpCommand = RampLiftCommand(&m_rampSubsystem).ToPtr();
  frc::SmartDashboard::PutData("Ramp Lift", liftUpCommand.get());
  m_assistController.RightBumper().Debounce(Operator::Assist::kDebouncePeriodLift).OnTrue(std::move(liftUpCommand));

  frc2::CommandPtr liftDownCommand = RampDropCommand(&m_rampSubsystem).ToPtr();
  frc::SmartDashboard::PutData("Ramp Down",liftDownCommand.get());
  m_assistController.LeftBumper().Debounce(Operator::Assist::kDebouncePeriodLift).OnTrue(std::move(liftDownCommand));
  #endif
  m_assistController.B().OnTrue(m_rampSubsystem.GetDefaultCommand());

  frc2::CommandPtr gatherCommand = RampGatherCommand(&m_rampSubsystem).ToPtr();
  frc::SmartDashboard::PutData("Ramp Gather", gatherCommand.get());
  m_assistController.A().WhileTrue(std::move(gatherCommand));

  frc2::CommandPtr launchUpRampCommand = RampLaunchCommand(&m_rampSubsystem,false,true).ToPtr();
  frc::SmartDashboard::PutData("Launch Up Ramp",launchUpRampCommand.get());
  m_assistController.Y().Debounce(Operator::Assist::kDebouncePeriodLaunch).OnTrue(std::move(launchUpRampCommand));

  frc2::CommandPtr launchDownRampCommand = RampLaunchCommand(&m_rampSubsystem,false,false).ToPtr();
  frc::SmartDashboard::PutData("Launch Down Ramp",launchDownRampCommand.get());
  m_assistController.X().Debounce(Operator::Assist::kDebouncePeriodLaunch).OnTrue(std::move(launchDownRampCommand));

  frc2::CommandPtr launchCommand = RampLaunchCommand(&m_rampSubsystem,true).ToPtr();
  frc::SmartDashboard::PutData("Launch No-Ramp",launchCommand.get());
  m_assistController.Start().Debounce(Operator::Assist::kDebouncePeriodLaunch).OnTrue(std::move(launchCommand));
#endif
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Currently returns a empty CommandPtr
#ifndef REMOVE_AUTO
  return m_chooser.GetSelected(); 
#else
  return nullptr;
#endif
}
#ifndef REMOVE_SWERVE
void RobotContainer::SetSwerveDefaultCommandXbox(frc2::CommandXboxController& control)
{
  SwerveDriveCommand driveCommand = SwerveDriveCommand(&m_swerveSubsystem,
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
    [&]{return control.GetLeftTriggerAxis() < 0.7;});
    driveCommand.SetDashboardFunc(
    [this](bool stopping){
      if(stopping)
      {
        frc::SmartDashboard::PutNumber("Speed",0);
        return;
      }
      frc::SmartDashboard::PutNumber("Speed",std::get<frc2::CommandXboxController>(m_driverController).RightBumper().Get() ? 
      Operator::Drive::kPercentDrivePrecision : 
      std::lerp(Operator::Drive::kPercentDriveLow,1,std::get<frc2::CommandXboxController>(m_driverController).GetRightTriggerAxis()));
    });
  m_swerveSubsystem.SetDefaultCommand(std::move(driveCommand).ToPtr()); 
}

void RobotContainer::SetSwerveDefaultCommandJoy(frc2::CommandJoystick& control)
{
  SwerveDriveCommand driveCommand = SwerveDriveCommand(&m_swerveSubsystem,
    [&]{return -control.GetThrottle() < Operator::Drive::kPrecisionThrottleThreshold ? 
      Operator::Drive::kPercentDrivePrecision * control.GetY() : 
      std::lerp(Operator::Drive::kPercentDriveLow,1,(-control.GetThrottle() - Operator::Drive::kPrecisionThrottleThreshold) / (1 - Operator::Drive::kPrecisionThrottleThreshold)) * control.GetY();},
    [&]{return -control.GetThrottle() < Operator::Drive::kPrecisionThrottleThreshold ? 
      Operator::Drive::kPercentDrivePrecision * control.GetX() : 
      std::lerp(Operator::Drive::kPercentDriveLow,1,(-control.GetThrottle() - Operator::Drive::kPrecisionThrottleThreshold) / (1 - Operator::Drive::kPrecisionThrottleThreshold)) * control.GetX();},
    [&]{return -control.GetThrottle() < Operator::Drive::kPrecisionThrottleThreshold ? 
      Operator::Drive::kPercentDrivePrecision * control.GetTwist() : 
      std::lerp(Operator::Drive::kPercentDriveLow,1,(-control.GetThrottle() - Operator::Drive::kPrecisionThrottleThreshold) / (1 - Operator::Drive::kPrecisionThrottleThreshold)) * control.GetTwist();},
    [&]{return control.GetRawButton(2);},
    [&]{return !control.GetRawButton(1);});
    driveCommand.SetDashboardFunc(
    [this](bool stopping){
      if(stopping)
      {
        frc::SmartDashboard::PutNumber("Speed",0);
        return;
      }
      frc::SmartDashboard::PutNumber("Speed",-std::get<frc2::CommandJoystick>(m_driverController).GetThrottle() < Operator::Drive::kPrecisionThrottleThreshold ? 
      Operator::Drive::kPercentDrivePrecision : 
      std::lerp(Operator::Drive::kPercentDriveLow,1,(-std::get<frc2::CommandJoystick>(m_driverController).GetThrottle() - Operator::Drive::kPrecisionThrottleThreshold) / (1 - Operator::Drive::kPrecisionThrottleThreshold)));
    });
  m_swerveSubsystem.SetDefaultCommand(std::move(driveCommand).ToPtr());
}
#endif 
#ifndef REMOVE_AUTO
void RobotContainer::GenerateSendable()
{
  std::filesystem::path dir =  frc::filesystem::GetDeployDirectory() + "/pathplanner";
  std::filesystem::path autos = dir.string() + "/autos";
  wpi::SmallVector<std::string,32> autoPaths;
  std::puts(autos.c_str());

  if(std::filesystem::directory_entry{autos}.exists())
  {
    for(std::filesystem::directory_entry const& entry : std::filesystem::directory_iterator{autos})
    {
      if(entry.is_directory()) { return; }
      if(!entry.path().string().ends_with(".auto")) { return; }
      autoPaths.emplace_back(entry.path().string().substr(0,entry.path().string().find('.')));
    }
  }

  
  m_commands.emplace_back(frc2::cmd::None());
  m_chooser.SetDefaultOption("None",m_commands.back().get());

  for(std::string const& entry : autoPaths)
  {
    m_commands.emplace_back(pathplanner::PathPlannerAuto(entry));
    m_chooser.AddOption(entry,m_commands.back().get());
  }
}
#endif