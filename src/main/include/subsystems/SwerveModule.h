#pragma once

#include "Constants.h"

#include <units/angle.h>
#include <units/velocity.h>

#include <frc/geometry/Rotation2d.h>

#include <rev/CANSparkMax.h>
#include <rev/AbsoluteEncoder.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>

class SwerveModule {
public:
    /**
     * Constructs an interface to a Swerve Module
     * 
     * @param drivingCANId The ID of the driving motor: Talon FX
     * @param turningCANId The ID of the turning motor: Spark Max
     * @param chassisAngularOffest The offset of the wheel on the motor in radians
    */
    SwerveModule(const int drivingCANId, const int turningCANId,
                                 const double chassisAngularOffset);
    //* @return A struct of the current driving velocity and rotational position
    frc::SwerveModuleState GetState();
    //* @return A struct of the current driving position and rotational position
    frc::SwerveModulePosition GetPosition();
    /**
     * Set the desired state of this swerve module
     * 
     * @param state A struct describing driving velocity and rotational position
    */
    void SetState(frc::SwerveModuleState state);
    //* Resets the driving encoders position to 0
    void ResetEncoders();
    //* Stops the movement of driving motor and turning motor
    void Stop();
private:
    /** 
     * Driving Motor: Talon FX
     * 
     * Falcon 500 that contains an intergated encoder used for velocity 
     * control of the driving wheels.
     * 
    */
    ctre::phoenix6::hardware::TalonFX m_drivingTalonFx;
    /** 
     * Turning Motor: Spark Max
     * 
     * NEO 550 connected to a Spark Max used for rotational position.
     * 
    */
    rev::CANSparkMax m_turningSparkMax;
    /** 
     * Rotation Encoder: Absolute Position
     * 
     * Through Bore Encoder connected to Spark Max of Turning Motor used
     * to report the current rotation of the wheels.
     * 
    */
    rev::SparkAbsoluteEncoder m_turningAbsoluteEncoder;
    /**
     * Turning PID Controller
     * 
     * Spark Max PID Controller used to move the Turning Motor to the 
     * current desired position, intergated into the Spark Max.
    */
    rev::SparkPIDController m_turningPIDController;
    /**
     * Wheel Offset
     * 
     * Chassis Angular Offset compared to their original positions on 
     * the robot, allowing all wheels to align.
    */
    units::radian_t m_chassisAngularOffset;
    /**
     * Desired Swerve Module State
     * 
     * The prevous Swerve Module State sent to this module, used for
     * debugging.
    */
    frc::SwerveModuleState m_desiredState{units::meters_per_second_t{0.0},
                                            frc::Rotation2d()};
};