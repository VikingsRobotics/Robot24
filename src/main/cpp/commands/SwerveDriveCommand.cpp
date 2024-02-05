#include "commands/SwerveDriveCommand.h"
#include <frc/MathUtil.h>
#include "Constants.h"
#include <frc/smartdashboard/smartdashboard.h>

SwerveDriveCommand::SwerveDriveCommand(SwerveSubsystem* subsystem,std::function<double(void)> xSpdFunc,
std::function<double(void)> ySpdFunc,std::function<double(void)> aSpdFunc) : m_subsystem{subsystem},
m_xSpdFunc{xSpdFunc},m_ySpdFunc{ySpdFunc},m_aSpdFunc{aSpdFunc} 
{
    AddRequirements(m_subsystem);
    SetName("Swerve Drive Command");
}

void SwerveDriveCommand::Execute() {
    double xSpeed = -m_xLimiter.Calculate(frc::ApplyDeadband<double>(m_xSpdFunc(),SwerveDrive::kDriveDeadband)) * SwerveDrive::kDriveMoveSpeedMax;
    double ySpeed = -m_yLimiter.Calculate(frc::ApplyDeadband<double>(m_ySpdFunc(),SwerveDrive::kDriveDeadband)) * SwerveDrive::kDriveMoveSpeedMax;
    double aSpeed = -m_aLimiter.Calculate(frc::ApplyDeadband<double>(m_aSpdFunc(),SwerveDrive::kDriveDeadband)) * SwerveDrive::kDriveAngleSpeedMax;

    wpi::array<frc::SwerveModuleState,4> moduleStates = SwerveDrive::kDriveKinematics.ToSwerveModuleStates(frc::ChassisSpeeds::FromFieldRelativeSpeeds( 
        units::meters_per_second_t{ xSpeed },units::meters_per_second_t{ ySpeed },units::radians_per_second_t{ aSpeed },m_subsystem->GetRotation2d()));
    for(int i = 0;i<moduleStates.size();++i)
    {
        frc::SmartDashboard::PutNumber("["+std::to_string(i)+"] speed",moduleStates.at(i).speed.value());
        frc::SmartDashboard::PutNumber("["+std::to_string(i)+"] angle",moduleStates.at(i).angle.Degrees().value());
    }
    m_subsystem->SetModulesState(&moduleStates);
}

void SwerveDriveCommand::End(bool interrupted) {
    m_subsystem->StopModules();
}