#pragma once

#include "Constants.h"

#include <rev/CANSparkMax.h>
#include <rev/AbsoluteEncoder.h>
#include <rev/SparkRelativeEncoder.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveModuleState.h>

class SwerveModule {
public:
    SwerveModule(int driveId, int angleId,bool driveReversed,bool angleReversed,double offset,std::string name);

    double GetDrivePosition();
    double GetAnglePosition();
    double GetDriveVelocity();
    double GetAngleVelocity();

    void ResetEncoders();

    frc::SwerveModuleState GetState();
    void SetState(frc::SwerveModuleState state);
    void Stop();
private:
    ctre::phoenix6::hardware::TalonFX  m_driveMotor;
    rev::CANSparkMax m_angleMotor;

    rev::SparkRelativeEncoder m_anglingEncoder;
    double m_offset;
    std::string m_name;

    frc::PIDController m_anglingPIDController{SwerveDrive::kPControl,SwerveDrive::kIControl,SwerveDrive::kDControl};
};