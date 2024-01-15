// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/CANSparkLowLevel.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {

constexpr int kDriverControllerPort = 0;

}  // namespace OperatorConstants

namespace CanBus
{
constexpr const char * kBusName = "rio"; // can be "", "rio",etc.
//Gryo
constexpr int kGyroId = 1;
//Motor id & invert
constexpr int kFLDriveMotorId = 2; //FL = front left
constexpr int kFLAngleMotorId = 3; 
constexpr bool kFLDriveInvert = false; 
constexpr bool kFLAngleInvert = false;
constexpr int kFRDriveMotorId = 4; //FR = front right
constexpr int kFRAngleMotorId = 5;
constexpr bool kFRDriveInvert = false; 
constexpr bool kFRAngleInvert = false;
constexpr int kBLDriveMotorId = 6; //BL = back left
constexpr int kBLAngleMotorId = 7;
constexpr bool kBLDriveInvert = false; 
constexpr bool kBLAngleInvert = false;
constexpr int kBRDriveMotorId = 8; //BR = back right
constexpr int kBRAngleMotorId = 9;
constexpr bool kBRDriveInvert = false; 
constexpr bool kBRAngleInvert = false;
//Motor info
constexpr rev::CANSparkLowLevel::MotorType kSparkType = rev::CANSparkLowLevel::MotorType::kBrushless;
constexpr int kSparkResolution = 42; //resolution = clicks per turn
constexpr int KTalonResolution = 2048;
} // namespace CanBus

namespace SwerveDrive
{
constexpr double kDriveDeadband = 0.05;
constexpr double kDriveMoveSpeedMax = 2; //meters per sec
constexpr double kDriveAngleSpeedMax = 0.5; //radians per sec

constexpr double kWheelDiameter = 0.0762; //meters
constexpr double kAngleGearRatio = 12;
//Angling PID control system
constexpr double kPControl = 5;
constexpr double kIControl = 0;
constexpr double kDControl = 0;
//Swerve Drive Kinematics, unfortunately can't be constexpr
//Motor distance from center in order fl, fr, bl, br
const frc::SwerveDriveKinematics<4> kDriveKinematics{ frc::Translation2d{+15_in,+15_in}, frc::Translation2d{+15_in,-15_in},
    frc::Translation2d{-15_in,+15_in}, frc::Translation2d{-15_in,-15_in} };
} // namespace SwerveDrive