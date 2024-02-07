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
    //auto -> units::meters_per_second_t & units::radians_per_second_t
    auto xSpeed = -m_xLimiter.Calculate(frc::ApplyDeadband(m_xSpdFunc(),SwerveDrive::kDriveDeadband)) * SwerveDrive::kDriveMoveSpeedMax;
    auto ySpeed = -m_yLimiter.Calculate(frc::ApplyDeadband(m_ySpdFunc(),SwerveDrive::kDriveDeadband)) * SwerveDrive::kDriveMoveSpeedMax;
    auto aSpeed = -m_aLimiter.Calculate(frc::ApplyDeadband(m_aSpdFunc(),SwerveDrive::kDriveDeadband)) * SwerveDrive::kDriveAngleSpeedMax;


    frc::SmartDashboard::PutNumber("X Joystick",m_xSpdFunc());
    frc::SmartDashboard::PutNumber("X Speed",xSpeed.value());
    frc::SmartDashboard::PutNumber("Y Speed",ySpeed.value());
    frc::SmartDashboard::PutNumber("A Speed",aSpeed.value());

    wpi::array<frc::SwerveModuleState,4> moduleStates = SwerveDrive::kDriveKinematics.ToSwerveModuleStates(frc::ChassisSpeeds::FromFieldRelativeSpeeds( 
        xSpeed, ySpeed, aSpeed,m_subsystem->GetRotation2d()));
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