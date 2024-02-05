#pragma once

#include "Constants.h"

#include <rev/CANSparkMax.h>
#include <rev/AbsoluteEncoder.h>
//#include <rev/SparkRelativeEncoder.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveModuleState.h>

class SwerveModule {
public:
    SwerveModule(const int drivingCANId, const int turningCANId,
                                 const double chassisAngularOffset);

    frc::SwerveModuleState GetState();
    frc::SwerveModulePosition GetPosition();
    void SetState(frc::SwerveModuleState state);
    void ResetEncoders();
    void Stop();
private:
    //rev::CANSparkMax m_drivingSparkMax;
    ctre::phoenix6::hardware::TalonFX m_drivingTalonFx;
    rev::CANSparkMax m_turningSparkMax;

    /*rev::SparkRelativeEncoder m_drivingEncoder =
        m_drivingSparkMax.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);*/
    rev::SparkAbsoluteEncoder m_turningAbsoluteEncoder;

    /*rev::SparkPIDController m_drivingPIDController =
        m_drivingSparkMax.GetPIDController();*/
    rev::SparkPIDController m_turningPIDController;

    double m_chassisAngularOffset = 0;
    frc::SwerveModuleState m_desiredState{units::meters_per_second_t{0.0},
                                            frc::Rotation2d()};
};