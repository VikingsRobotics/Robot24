#include "subsystems/SwerveSubsystem.h"
#include <math.h>
#include <frc/smartdashboard/smartdashboard.h>
#include <frc/smartdashboard/smartdashboard.h>
#include <string>

SwerveSubsystem::SwerveSubsystem() : m_odometry{ SwerveDrive::kDriveKinematics,frc::Rotation2d{ units::radian_t{0} }, {m_frontLeft.GetPosition(),m_frontRight.GetPosition(),m_backLeft.GetPosition(),m_backRight.GetPosition()}} { 
    m_gryo.Reset(); 
    SetName("Swerve Drive Subsystem");
    frc::SmartDashboard::PutData("Driver",this);
#ifdef DEBUG_SWERVE_MODULE
    frc::SmartDashboard::PutString("["+m_frontLeft.GetName()+"] Debug State","Info");
    frc::SmartDashboard::PutString("["+m_frontRight.GetName()+"] Debug State","Info");
    frc::SmartDashboard::PutString("["+m_backLeft.GetName()+"] Debug State","Info");
    frc::SmartDashboard::PutString("["+m_backRight.GetName()+"] Debug State","Info");
#endif //DEBUG_SWEVE_MODULE
}

void SwerveSubsystem::Periodic() { 
    m_odometry.Update(GetRotation2d(),{m_frontLeft.GetPosition(),m_frontRight.GetPosition(),m_backLeft.GetPosition(),m_backRight.GetPosition()});
#ifdef DEBUG_SWERVE_MODULE
    //Get debug state
    std::string fl = frc::SmartDashboard::GetString("["+m_frontLeft.GetName()+"] Debug State","Info");
    std::string fr = frc::SmartDashboard::GetString("["+m_frontRight.GetName()+"] Debug State","Info");
    std::string bl = frc::SmartDashboard::GetString("["+m_backLeft.GetName()+"] Debug State","Info");
    std::string br = frc::SmartDashboard::GetString("["+m_backRight.GetName()+"] Debug State","Info");
    //Transform state to uppercase
    std::transform(fl.begin(), fl.end(), fl.begin(),
    [](unsigned char c){ return std::toupper(c); });
    std::transform(fr.begin(), fr.end(), fr.begin(),
    [](unsigned char c){ return std::toupper(c); });
    std::transform(bl.begin(), bl.end(), bl.begin(),
    [](unsigned char c){ return std::toupper(c); });
    std::transform(br.begin(), br.end(), br.begin(),
    [](unsigned char c){ return std::toupper(c); });
    //Use debug state
    if(fl != "INFO")
        if(fl != "ACTION")
            m_frontLeft.Debug(SwerveModule::DebugType::Disable);
        else
            m_frontLeft.Debug(SwerveModule::DebugType::Action);
    else
        m_frontLeft.Debug(SwerveModule::DebugType::Info);
    
    if(fr != "INFO")
        if(fr != "ACTION")
            m_frontRight.Debug(SwerveModule::DebugType::Disable);
        else
            m_frontRight.Debug(SwerveModule::DebugType::Action);
    else
        m_frontRight.Debug(SwerveModule::DebugType::Info);
    
    if(bl != "INFO")
        if(bl != "ACTION")
            m_backLeft.Debug(SwerveModule::DebugType::Disable);
        else
            m_backLeft.Debug(SwerveModule::DebugType::Action);
    else
        m_backLeft.Debug(SwerveModule::DebugType::Info);
    
    if(br != "INFO")
        if(br != "ACTION")
            m_backRight.Debug(SwerveModule::DebugType::Disable);
        else
            m_backRight.Debug(SwerveModule::DebugType::Action);
    else
        m_backRight.Debug(SwerveModule::DebugType::Info);
#endif //DEBUG_SWERVE_MODULE
}

void SwerveSubsystem::ZeroHeading() { m_gryo.Reset(); }
double SwerveSubsystem::GetHeading() { return std::fmod(m_gryo.GetAngle(),360); }

frc::Rotation2d SwerveSubsystem::GetRotation2d() { return frc::Rotation2d{ units::degree_t{ GetHeading() } }; }

frc::Pose2d SwerveSubsystem::GetPose2d() { return m_odometry.GetPose(); }
void SwerveSubsystem::ResetOdometry(frc::Pose2d pose) {
    m_odometry.ResetPosition(GetRotation2d(),{m_frontLeft.GetPosition(),m_frontRight.GetPosition(),m_backLeft.GetPosition(),m_backRight.GetPosition()},pose);
}

void SwerveSubsystem::StopModules() {
    m_frontLeft.Stop();
    m_frontRight.Stop();
    m_backLeft.Stop();
    m_backRight.Stop();
}
void SwerveSubsystem::SetModulesState(wpi::array<frc::SwerveModuleState,4>* states) {
    SwerveDrive::kDriveKinematics.DesaturateWheelSpeeds(states,units::meters_per_second_t{ SwerveDrive::kPhysicalMoveMax });
    m_frontLeft.SetState((*states)[0]);
    m_frontRight.SetState((*states)[1]);
    m_backLeft.SetState((*states)[2]);
    m_backRight.SetState((*states)[3]);
}