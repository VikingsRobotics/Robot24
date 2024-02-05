// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/CANSparkMax.h>
#include <rev/AbsoluteEncoder.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <networktables/StructTopic.h>

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
constexpr int kGyroId = 1;
constexpr int kFLDriveMotorId = 2; //FL = front left
constexpr int kFLAngleMotorId = 3; 
constexpr int kFRDriveMotorId = 4; //FR = front right
constexpr int kFRAngleMotorId = 5;
constexpr int kBLDriveMotorId = 6; //BL = back left
constexpr int kBLAngleMotorId = 7;
constexpr int kBRDriveMotorId = 8; //BR = back right
constexpr int kBRAngleMotorId = 9;
//Rev neo 550 info
constexpr rev::CANSparkMax::MotorType kSparkMotorType = rev::CANSparkMax::MotorType::kBrushless;
constexpr rev::SparkAbsoluteEncoder::Type kSparkAbsEncoderType = rev::SparkAbsoluteEncoder::Type::kDutyCycle;
constexpr bool kInvertEncoder = true;


constexpr int kSparkResolution = 42; //resolution = clicks per rotation
//CRTE falcon 500 info
constexpr int KTalonResolution = 2048;
} // namespace CanBus

namespace SwerveDrive
{
//Volt feedforward
constexpr units::volt_t kStaticVoltage = units::volt_t{0.149};
constexpr units::volt_t kVelocityVoltage = units::volt_t{8.9};
constexpr double kVelocityPControl = 0.1;
//Wheel Measurement
constexpr units::inch_t kWheelDiameter = units::inch_t{3};
constexpr units::meter_t kWheelCircumeter = kWheelDiameter * 2 * std::numbers::pi;

//Gear Ratio
constexpr double kDriveGearRatio = (1/5.08);
constexpr double kAngleGearRatio = 2 * std::numbers::pi;
//Angling PID control system
constexpr double kPControl = 1;

//Swerve Drive Kinematics, unfortunately can't be constexpr
//Motor distance from center in order fl, fr, bl, br
const frc::SwerveDriveKinematics<4> kDriveKinematics{ frc::Translation2d{+15_in,+15_in}, frc::Translation2d{+15_in,-15_in},
    frc::Translation2d{-15_in,+15_in}, frc::Translation2d{-15_in,-15_in} };
} // namespace SwerveDrive