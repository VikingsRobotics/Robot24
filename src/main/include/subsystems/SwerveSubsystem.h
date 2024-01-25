#pragma once

#include "subsystems/SwerveModule.h"
#include "Constants.h"

#include <ctre/phoenix6/Pigeon2.hpp>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

class SwerveSubsystem : public frc2::SubsystemBase {
public:
  SwerveSubsystem();
  void Periodic() override;

  void ZeroHeading();
  double GetHeading();
  frc::Rotation2d GetRotation2d();

  frc::Pose2d GetPose2d();
  void ResetOdometry(frc::Pose2d pose);

  void StopModules();
  void SetModulesState(wpi::array<frc::SwerveModuleState,4>* states);
private:
  ctre::phoenix6::hardware::Pigeon2 m_gryo{CanBus::kGyroId,CanBus::kBusName};

  SwerveModule m_frontLeft{CanBus::kFLDriveMotorId,CanBus::kFLAngleMotorId,CanBus::kFLDriveInvert,CanBus::kFLAngleInvert,-std::numbers::pi * 1/2,"Front Left"};
  SwerveModule m_frontRight{CanBus::kFRDriveMotorId,CanBus::kFRAngleMotorId,CanBus::kFRDriveInvert,CanBus::kFRAngleInvert,0,"Front Right"};
  SwerveModule m_backLeft{CanBus::kBLDriveMotorId,CanBus::kBLAngleMotorId,CanBus::kBLDriveInvert,CanBus::kBLAngleInvert,std::numbers::pi * 1/2,"Back Left"};
  SwerveModule m_backRight{CanBus::kBRDriveMotorId,CanBus::kBRAngleMotorId,CanBus::kBRDriveInvert,CanBus::kBRAngleInvert,std::numbers::pi,"Back Right"};

  wpi::array<frc::SwerveModulePosition,4> m_encoderPositions{ wpi::empty_array };
  
  frc::SwerveDriveOdometry<4> m_odometry{ SwerveDrive::kDriveKinematics,frc::Rotation2d{ units::radian_t{0} }, m_encoderPositions};
};