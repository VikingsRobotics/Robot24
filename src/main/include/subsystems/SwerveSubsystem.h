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

  SwerveModule m_frontLeft{ CanBus::kFLDriveMotorId,  CanBus::kFLAngleMotorId,  -std::numbers::pi / 2.0 };
  SwerveModule m_frontRight{CanBus::kFRDriveMotorId,  CanBus::kFRAngleMotorId,  0                       };
  SwerveModule m_backLeft{  CanBus::kBLDriveMotorId,  CanBus::kBLAngleMotorId,  std::numbers::pi        };
  SwerveModule m_backRight{ CanBus::kBRDriveMotorId,  CanBus::kBRAngleMotorId,  std::numbers::pi / 2.0  };

  frc::SwerveDriveOdometry<4> m_odometry;
};