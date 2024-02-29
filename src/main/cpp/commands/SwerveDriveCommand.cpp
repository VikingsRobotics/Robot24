#include "commands/SwerveDriveCommand.h"

#include "Constants.h"

#include <utility>

#include <frc/MathUtil.h>
#include <units/time.h>
#include <frc/smartdashboard/SmartDashboard.h>
#ifndef REMOVE_SWERVE
SwerveDriveCommand::SwerveDriveCommand(SwerveSubsystem* const subsystem,std::function<double(void)> xSpdFunc,
    std::function<double(void)> ySpdFunc,std::function<double(void)> aSpdFunc,std::function<bool(void)> brakeFunc,
    std::function<bool(void)> fieldFunc,double rateLimit) : m_subsystem{subsystem},
m_xSpdFunc{std::move(xSpdFunc)},m_ySpdFunc{std::move(ySpdFunc)},m_aSpdFunc{std::move(aSpdFunc)},m_brakeFunc{std::move(brakeFunc)},
m_fieldFunc{std::move(fieldFunc)},m_xLimiter{rateLimit / 1_s},m_yLimiter{rateLimit / 1_s},m_aLimiter{rateLimit / 1_s}
{
    // Adds to list of subsystem requirement so that only one command has access
    AddRequirements(m_subsystem);
    // Set our own name
    SetName("Swerve Drive Command");
}

void SwerveDriveCommand::Execute() {
    // Allow us to upload to dashboard
    if(m_dashboard) { m_dashboard(false); }
    // Check if we are currently braking
    if(m_brakeFunc())
    {
        // Call subsystem to break, it calls set modules
        m_subsystem->Brake();
        // Make sure to early return, don't do extra work
        return;
    }
    // Calculate the speed after deadbanding the input and multiplying by teleop speed
    auto xSpeed = -m_xLimiter.Calculate(frc::ApplyDeadband(m_xSpdFunc(),Operator::Drive::kDriveDeadband,1.0)) * Operator::Drive::kDriveMoveSpeedMax;
    auto ySpeed = -m_yLimiter.Calculate(frc::ApplyDeadband(m_ySpdFunc(),Operator::Drive::kDriveDeadband,1.0)) * Operator::Drive::kDriveMoveSpeedMax;
    auto aSpeed = -m_aLimiter.Calculate(frc::ApplyDeadband(m_aSpdFunc(),Operator::Drive::kDriveAngleDeadband,1.0)) * Operator::Drive::kDriveAngleSpeedMax;
    // Puts the speed into the kinematics to get states
    wpi::array<frc::SwerveModuleState,4> moduleStates = Swerve::System::kDriveKinematics.ToSwerveModuleStates( m_fieldFunc() ?
        frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, aSpeed,m_subsystem->GetRotation2d()) :
        frc::ChassisSpeeds(xSpeed,ySpeed,aSpeed));
    frc::SmartDashboard::PutBoolean("Field",m_fieldFunc());
    frc::SmartDashboard::PutNumber("Robot Heading",m_subsystem->GetHeading().value());
    // Loop all the state and display them to the dashboard
    for(size_t i = 0;i<moduleStates.size();++i)
    {
        frc::SmartDashboard::PutNumber("["+std::to_string(i)+"] speed",moduleStates.at(i).speed.value());
        frc::SmartDashboard::PutNumber("["+std::to_string(i)+"] angle",moduleStates.at(i).angle.Degrees().value());
    }
    // Send the states to the swerve systems for the real world
    m_subsystem->SetModulesState(moduleStates);
}

void SwerveDriveCommand::End(bool interrupted) {
    if(m_dashboard) { m_dashboard(true); }
    // Tidy everything up, don't want the robot to keep moving
    m_subsystem->StopModules();
}

void SwerveDriveCommand::SetDashboardFunc(std::function<void(bool)> dashboard)
{
    m_dashboard = std::move(dashboard);
}
#endif