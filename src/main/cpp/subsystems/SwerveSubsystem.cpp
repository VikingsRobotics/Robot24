#include "subsystems/SwerveSubsystem.h"

#include <math.h>

#include <frc/smartdashboard/smartdashboard.h>
#include <frc/geometry/Rotation2d.h>

#include <units/velocity.h>
#include <units/angle.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>
#include <frc/DriverStation.h>


SwerveSubsystem::SwerveSubsystem() : m_odometry{ Swerve::System::kDriveKinematics,frc::Rotation2d{ units::radian_t{0} }, {m_frontLeft.GetPosition(),m_frontRight.GetPosition(),m_backLeft.GetPosition(),m_backRight.GetPosition()}} { 
    // Make sure the current rotation is zero
    m_gryo.Reset(); 
    // Set our own name
    SetName("Swerve Drive Subsystem");
    // Publishes it to the dashboard
    frc::SmartDashboard::PutData("Driver",this);

    pathplanner::AutoBuilder::configureHolonomic(
        [this]{return GetPose2d();},
        [this](frc::Pose2d pose){ResetOdometry(pose);},
        [this]{return GetCurrentSpeeds();},
        [this](frc::ChassisSpeeds speed){Drive(speed);},
        pathplanner::HolonomicPathFollowerConfig{
            pathplanner::PIDConstants{Swerve::Auto::kPTranslationController,0,0},
            pathplanner::PIDConstants{Swerve::Auto::kPRotationController,0,0},
            Swerve::Mechanism::kPhysicalMoveMax,
            Swerve::System::kWheelBase,
            pathplanner::ReplanningConfig{}
        },
        []{
            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this
    );

    pathplanner::PathPlannerLogging::setLogActivePathCallback([this](auto poses) {
        m_field.GetObject("path")->SetPoses(poses);
    });

    frc::SmartDashboard::PutData("Field", &m_field);
}

void SwerveSubsystem::Periodic() { 
    // Tracks robot position using the position of swerve modules and gryo rotation
    m_odometry.Update(GetRotation2d(),{m_frontLeft.GetPosition(),m_frontRight.GetPosition(),m_backLeft.GetPosition(),m_backRight.GetPosition()});
}

void SwerveSubsystem::InitSendable(wpi::SendableBuilder& builder)
{
    frc2::SubsystemBase::InitSendable(builder);
    builder.AddDoubleProperty("Heading",[this]{return GetHeading().value();},nullptr);
    builder.AddDoubleProperty("Speed X",[this]{return GetCurrentSpeeds().vx.value();},nullptr);
    builder.AddDoubleProperty("Speed Y",[this]{return GetCurrentSpeeds().vy.value();},nullptr);
    builder.AddDoubleProperty("Speed Theta",[this]{return GetCurrentSpeeds().omega.convert<units::degrees_per_second>().value();},nullptr);
}

void SwerveSubsystem::ZeroHeading() { m_gryo.Reset(); }

units::degree_t SwerveSubsystem::GetHeading() { 
    double angle = m_gryo.GetAngle();
    return units::degree_t{ std::fmod(angle < 0 ? -angle : angle ,360.0) }; 
}

frc::Rotation2d SwerveSubsystem::GetRotation2d() { return frc::Rotation2d{ GetHeading() }; }

frc::Pose2d SwerveSubsystem::GetPose2d() { return m_odometry.GetPose(); }

void SwerveSubsystem::ResetOdometry(frc::Pose2d pose) {
    // Resets pose but still requires the current state of swerve module and gryo rotation
    m_odometry.ResetPosition(GetRotation2d(),{m_frontLeft.GetPosition(),m_frontRight.GetPosition(),m_backLeft.GetPosition(),m_backRight.GetPosition()},pose);
    m_field.SetRobotPose(GetPose2d());
}

void SwerveSubsystem::StopModules() {
    // Calls every swerve modules Stop function
    m_frontLeft.Stop();
    m_frontRight.Stop();
    m_backLeft.Stop();
    m_backRight.Stop();
}

void SwerveSubsystem::SetModulesState(wpi::array<frc::SwerveModuleState,4> states) {
    // Make sure that we are under max speed
    Swerve::System::kDriveKinematics.DesaturateWheelSpeeds(&states,Swerve::Mechanism::kPhysicalMoveMax);
    // Calls every swerve modules SetStates function
    m_frontLeft.SetState(states[0]);
    m_frontRight.SetState(states[1]);
    m_backLeft.SetState(states[2]);
    m_backRight.SetState(states[3]);
}

frc::ChassisSpeeds SwerveSubsystem::GetCurrentSpeeds()
{
    return Swerve::System::kDriveKinematics.ToChassisSpeeds(m_frontLeft.GetState(),m_frontRight.GetState(),m_backLeft.GetState(),m_backRight.GetState());
}

void SwerveSubsystem::Drive(frc::ChassisSpeeds speed)
{
    wpi::array<frc::SwerveModuleState,4> states = Swerve::System::kDriveKinematics.ToSwerveModuleStates(speed);
    SetModulesState(states);
}

void SwerveSubsystem::Brake()
{
    m_frontLeft.SetState(frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
    m_frontRight.SetState(frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
    m_backLeft.SetState(frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
    m_backRight.SetState(frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
}