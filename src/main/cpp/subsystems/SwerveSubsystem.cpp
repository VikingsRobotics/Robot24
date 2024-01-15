#include "subsystems/SwerveSubsystem.h"
#include <math.h>

SwerveSubsystem::SwerveSubsystem() { m_gryo.Reset(); }

void SwerveSubsystem::Periodic() { 
    m_encoderPositions[0] = { units::meter_t{ m_frontLeft.GetDrivePosition() }, frc::Rotation2d{ units::radian_t{ m_frontLeft.GetAnglePosition() } } };
    m_encoderPositions[1] = { units::meter_t{ m_frontRight.GetDrivePosition() }, frc::Rotation2d{ units::radian_t{ m_frontRight.GetAnglePosition() } } };
    m_encoderPositions[2] = { units::meter_t{ m_backLeft.GetDrivePosition() }, frc::Rotation2d{ units::radian_t{ m_backLeft.GetAnglePosition() } } };
    m_encoderPositions[3] = { units::meter_t{ m_backRight.GetDrivePosition() }, frc::Rotation2d{ units::radian_t{ m_backRight.GetAnglePosition() } } };
    m_odometry.Update(GetRotation2d(),m_encoderPositions);
}

void SwerveSubsystem::ZeroHeading() { m_gryo.Reset(); }
double SwerveSubsystem::GetHeading() { return std::fmod(m_gryo.GetAngle(),360); }

frc::Rotation2d SwerveSubsystem::GetRotation2d() { return frc::Rotation2d{ units::degree_t{ GetHeading() } }; }

frc::Pose2d SwerveSubsystem::GetPose2d() { return m_odometry.GetPose(); }
void SwerveSubsystem::ResetOdometry(frc::Pose2d pose) {
    m_odometry.ResetPosition(GetRotation2d(),m_encoderPositions,pose);
}

void SwerveSubsystem::StopModules() {
    m_frontLeft.Stop();
    m_frontRight.Stop();
    m_backLeft.Stop();
    m_backRight.Stop();
}
void SwerveSubsystem::SetModulesState(wpi::array<frc::SwerveModuleState,4>* states) {
    SwerveDrive::kDriveKinematics.DesaturateWheelSpeeds(states,5_mps);
    m_frontLeft.SetState((*states)[0]);
    m_frontRight.SetState((*states)[1]);
    m_backLeft.SetState((*states)[2]);
    m_backRight.SetState((*states)[3]);
}