#include "commands/SwerveTesterCommand.h"
#include <frc/smartdashboard/SmartDashboard.h>

SwerveTesterCommand::SwerveTesterCommand(SwerveSubsystem* subsystem,bool bDirection) : m_subsystem(subsystem), m_speed(bDirection ? 5 : 0), m_direction(m_direction) 
{
    AddRequirements(subsystem);
}

void SwerveTesterCommand::Initialize()
{
    m_speeder.Reset(m_direction ? 5 : 0);
}

void SwerveTesterCommand::Execute()
{
    m_speed = m_speeder.Calculate(m_direction ? 0 : 5);
    
    wpi::array<frc::SwerveModuleState,4> moduleStates = SwerveDrive::kDriveKinematics.ToSwerveModuleStates(frc::ChassisSpeeds::FromFieldRelativeSpeeds( 
        units::meters_per_second_t { -m_speed }, 0_mps, 0_rad_per_s,m_subsystem->GetRotation2d()));
    for(int i = 0;i<moduleStates.size();++i)
    {
        frc::SmartDashboard::PutNumber("["+std::to_string(i)+"] speed",moduleStates.at(i).speed.value());
        frc::SmartDashboard::PutNumber("["+std::to_string(i)+"] angle",moduleStates.at(i).angle.Degrees().value());
    }
    m_subsystem->SetModulesState(&moduleStates);
}

bool SwerveTesterCommand::IsFinished()
{
    return m_direction ? (m_speed < 0.001) : (m_speed > 4.999);
}

void SwerveTesterCommand::End(bool bInterupted)
{
    m_subsystem->StopModules();
}