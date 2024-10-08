// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>
#include <units/length.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/time.h>

#include <frc/PneumaticsModuleType.h>

#include <frc/geometry/Transform2d.h>

#include <rev/CANSparkMax.h>
#include <rev/AbsoluteEncoder.h>

#include <frc/kinematics/SwerveDriveKinematics.h>

#include <frc/trajectory/TrapezoidProfile.h>

#include <math.h>
#include <numbers>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

// regionDebug 

//#define REMOVE_SWERVE 1
//#define REMOVE_RAMP 1
//#define REMOVE_SOLENOID 1
//#define REMOVE_AUTO 1

// endregionDebug

namespace Operator {
//* USB Port to the first controller connected to PC
constexpr int kDriverControllerPort = 0;

constexpr int kAssistControllerPort = 1;
    namespace Drive
    {
    //* Prevents controller from running when under very low values
    constexpr double kDriveDeadband = 0.05;

    constexpr double kDriveAngleDeadband = 0.4;
    //* TeleOp drivers controlling movement speed
    constexpr units::meters_per_second_t kDriveMoveSpeedMax = 3.0_mps;
    //* TeleOp drivers lowest normal speed
    constexpr units::meters_per_second_t kDriveMoveSpeedLow = 1.5_mps;
    //* Percent that the low speed on the range max speed
    constexpr double kPercentDriveLow = kDriveMoveSpeedLow/kDriveMoveSpeedMax;
    //* Precision drivers lowest speed
    constexpr units::meters_per_second_t kDrivePrecision = 0.6_mps;
    //* Percent that the precision speed on the range max speed
    constexpr double kPercentDrivePrecision = kDrivePrecision/kDriveMoveSpeedMax;
    //* When the throttle axis transforms into precision control on joystick
    constexpr double kPrecisionThrottleThreshold = -0.6;
    //* TeleOp drivers controlling angular speed
    constexpr units::radians_per_second_t kDriveAngleSpeedMax = 3.0_rad_per_s;
    } // namespace Drive
    namespace Assist
    {
        constexpr units::second_t kDebouncePeriodLift = 0.2_s;

    } // namespace Assist
}  // namespace OperatorConstants
//* namespace for device IDs
namespace Device
{
//* CTRE: Bus name needed for constructors
constexpr const char * kBusName = "rio";
//* (CANBUS) ID for the Pigeon 2.0 (CTRE)
constexpr int kGyroId = 10;
//* (CANBUS) ID for the Front Left Falcon 500 driving motor (CTRE)
constexpr int kFLDriveMotorId = 2;
//* (CANBUS) ID for the Front Left NEO 550 turning motor (REV)
constexpr int kFLAngleMotorId = 3; 
//* (CANBUS) ID for the Front Right Falcon 500 driving motor (CTRE)
constexpr int kFRDriveMotorId = 4;
//* (CANBUS) ID for the Front Right NEO 550 turning motor (REV)
constexpr int kFRAngleMotorId = 5;
//* (CANBUS) ID for the Back Left Falcon 500 driving motor (CTRE)
constexpr int kBLDriveMotorId = 6;
//* (CANBUS) ID for the Back Left NEO 550 turning motor (REV)
constexpr int kBLAngleMotorId = 7;
//* (CANBUS) ID for the Back Right Falcon 500 driving motor (CTRE)
constexpr int kBRDriveMotorId = 8;
//* (CANBUS) ID for the Back Right NEO 550 turning motor (REV)
constexpr int kBRAngleMotorId = 9;

constexpr int kSweeperMotorId = 1;

constexpr int kFeederMotorId = 0;

constexpr int kTopRightMotorId = 2;

constexpr int kTopLeftMotorId = 3;

constexpr int kPneumaticForwardId = 0;

constexpr int kPneumaticBackwardId = 1;
    //* subnamespace for internal device info
    namespace Internal
    {
    //* Default motor type used for spark max motors
    constexpr rev::CANSparkMax::MotorType kSparkMotorType = rev::CANSparkMax::MotorType::kBrushless;
    //* Default encoder type used for spark absolute encoders
    constexpr rev::SparkAbsoluteEncoder::Type kSparkAbsEncoderType = rev::SparkAbsoluteEncoder::Type::kDutyCycle;

    constexpr rev::SparkRelativeEncoder::Type kSparkRelEncoderType = rev::SparkRelativeEncoder::Type::kHallSensor;
    //* Invert absolute encoder to match direction of motor movement
    constexpr bool kInvertEncoder = true;
    //* @deprecated How many encoder ticks are in one rotation
    constexpr int kSparkResolution = 42;
    //* @deprecated How many encoder ticks are in one rotation
    constexpr int KTalonResolution = 2048;
    //* Type of control module for the pneumatics
    constexpr frc::PneumaticsModuleType kPneumaticType = frc::PneumaticsModuleType::CTREPCM;
    }
} // namespace CanBus
namespace Ramp
{

    constexpr double kRetreatSpeed{0.55};

    constexpr double kRetreatSpeedSlow{0.20};

    constexpr double kLaunchSpeed{0.515};

    constexpr double kLauncherSpeedLow{0.55};

    constexpr double kLauncherSpeedHigh{0.95};

    constexpr units::second_t kRetreatTime{0.5};

    constexpr units::second_t kVelocityTime{0.25};

    constexpr units::second_t kLaunchTime{2};

    constexpr units::second_t kLiftTime{1};

    constexpr units::second_t kDownTime{0};
}
//* namespace containing all swerve module constants
namespace Swerve
{
    //* subnamespace containing all mechanism constants
    namespace Mechanism
    {
    //* 990 motor teeth to 195 wheel teeth, converts motor rotations to wheel rotations
    constexpr double kDriveGearRatio{990/195.0};
    //* 1 rot to 2 pi radians, converts motor rotations to radians
    constexpr double kAngleGearRatio{2 * std::numbers::pi};
    //* Max 108 rotations per sec from driving motor, without gear ratios
    constexpr units::turns_per_second_t kDriveRps{108};
    //* Wheel diameter in inches, 3
    constexpr units::inch_t kWheelDiameter{3};
    //* Wheel circumference in meters, ~0.24
    constexpr units::meter_t kWheelCircumference{kWheelDiameter * std::numbers::pi};
    //* Converts meters to rotations, mps to rps, and mps^2 to rps^2, rotations = (meterTarget * gearRatio) / WheelCircumferenceMeters
    constexpr double kDriveSpeedToTurns{kDriveGearRatio/kWheelCircumference.value()};
    //* Converts rotations to meters, rps to mps, and rps^2 to mps^2, meters = (WheelCircumferenceMeters * rotationsTarget) / gearRatio
    constexpr double kTurnsToDriveSpeed{kWheelCircumference.value()/kDriveGearRatio};
    //* Min voltage required for driving motor to begin moving
    constexpr units::volt_t kStaticVoltage{0.15};
    //* kV for feedforward, target rotation is multipled kV and added to velocity control
    constexpr units::volt_t kVelocityVoltage{12/kDriveRps.value()};
    //* Max speed the wheel move, used to normialize swerve modules speeds to maintain control
    constexpr units::meters_per_second_t kPhysicalMoveMax{kDriveRps.value() * kWheelCircumference.value() / kDriveGearRatio};
    } // Mechanism
    //* subnamespace contain all PID control sytems constants and kinematics constants
    namespace System
    {
    //* Proportional term for velocity control system used by driving wheels
    constexpr double kVelocityPControl{0.1};
    //* Proportional term for position control system used by turning wheels
    constexpr double kTurningPControl{1};
    //* Distance between centers of right and left wheels on robot
    constexpr units::inch_t kTrackWidth{15};
    //* Distance between centers of front and back wheels on robot
    constexpr units::inch_t kWheelBase{15};
    /** 
    * Swerve Kinematics, using positions of wheels to calculate desired states for swerve modules
    * 
    * Inputted from left front, right front, left back, right back
    */
    extern frc::SwerveDriveKinematics<4> kDriveKinematics;
    } // namespace System
    namespace Auto
    {
    
    constexpr double kPTranslationController = 1;

    constexpr double kPRotationController = 1;

    constexpr units::meters_per_second_t kMaxSpeed = 5_mps;

    constexpr units::meters_per_second_squared_t kMaxAcceleration = 3_mps_sq;

    constexpr units::radians_per_second_t kMaxAngularSpeed = 540_deg_per_s;

    constexpr units::radians_per_second_squared_t kMaxAngularAcceleration = 720_deg_per_s_sq;
    }
} // namespace Swerve