#pragma once

#include "subsystems/SwerveModule.h"
#include "Constants.h"

#include <units/angle.h>

#include <ctre/phoenix6/Pigeon2.hpp>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

class SwerveSubsystem : public frc2::SubsystemBase {
public:
  /**
   * Constructs a Swerve Subsystem
   * 
   * Provides functions to control swerve module movement
  */
  SwerveSubsystem();
  //* Subsystem overloaded function: executed every command schedule pass
  void Periodic() override;
  //* Sets the current rotation as 0
  void ZeroHeading();
  //* Gets the current rotation in degrees
  units::degree_t GetHeading();
  //* Gets the current rotation in a Rotation2d
  frc::Rotation2d GetRotation2d();
  //* Gets the current position and rotation from the odometry
  frc::Pose2d GetPose2d();
  /**
   * Sets the current position and rotation in odometry
   * 
   * @param pose the pose that odometry will be set to
  */
  void ResetOdometry(frc::Pose2d pose);
  //* Stops all of the motors from moving
  void StopModules();
  /**
   * Sets the desired state for the swerve modules
   * 
   * @param states* pointer to array of module state that are desired
  */
  void SetModulesState(wpi::array<frc::SwerveModuleState,4>* states);

  void Brake();
private:
  //* Gryo used for odometry and for field centric control
  ctre::phoenix6::hardware::Pigeon2 m_gryo{Device::kGyroId,Device::kBusName};
  //* Front Left module
  SwerveModule m_frontLeft {Device::kFLDriveMotorId,  Device::kFLAngleMotorId,  -std::numbers::pi / 2.0 };
  //* Front Right module
  SwerveModule m_frontRight{Device::kFRDriveMotorId,  Device::kFRAngleMotorId,  0                       };
  //* Back Left module
  SwerveModule m_backLeft  {Device::kBLDriveMotorId,  Device::kBLAngleMotorId,  std::numbers::pi        };
  //* Back Right module
  SwerveModule m_backRight {Device::kBRDriveMotorId,  Device::kBRAngleMotorId,  std::numbers::pi / 2.0  };
  //* Track the position of the robot using wheel position and gryo rotation
  frc::SwerveDriveOdometry<4> m_odometry;
};