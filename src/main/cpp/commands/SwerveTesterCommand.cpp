#include "commands/SwerveTesterCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

SwerveTesterCommand::SwerveTesterCommand(SwerveSubsystem* subsystem,RampDirection bDirection,units::second_t time) : 
    m_subsystem(subsystem), m_speed(bDirection ? Swerve::Mechanism::kPhysicalMoveMax.value() : 0), m_direction(bDirection),m_speeder(Swerve::Mechanism::kPhysicalMoveMax.value() / time) 
{
    // Adds to list of subsystem requirement so that only one command has access
    AddRequirements(subsystem);
}

void SwerveTesterCommand::Initialize()
{
    // Resets the position so the command can be used multiple times
    m_speeder.Reset(m_direction ? Swerve::Mechanism::kPhysicalMoveMax.value() : 0);
}

void SwerveTesterCommand::Execute()
{
    // Gets the speed for this frame
    m_speed = m_speeder.Calculate(m_direction ? 0 : 5);
    // Puts the speed into the kinematics to get states
    wpi::array<frc::SwerveModuleState,4> moduleStates = Swerve::System::kDriveKinematics.ToSwerveModuleStates(frc::ChassisSpeeds( 
        units::meters_per_second_t { -m_speed }, 0_mps, 0_rad_per_s));
    // Loop all the state and display them to the dashboard
    for(size_t i = 0;i<moduleStates.size();++i)
    {
        frc::SmartDashboard::PutNumber("["+std::to_string(i)+"] speed",moduleStates.at(i).speed.value());
    }
    // Send the states to the swerve systems for the real world
    m_subsystem->SetModulesState(moduleStates);
}

bool SwerveTesterCommand::IsFinished()
{
    // If the current speed is within 0.001 of their end goal speed, we unschedule ourself
    return m_direction ? (m_speed < 0.001) : (m_speed > Swerve::Mechanism::kPhysicalMoveMax.value() - 0.001);
}

void SwerveTesterCommand::End(bool bInterupted)
{
    // Tidy everything up, don't want the robot to keep moving
    m_subsystem->StopModules();
}