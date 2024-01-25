#include "subsystems/SwerveModule.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <iostream>
#include <math.h>

SwerveModule::SwerveModule(int driveId,int angleId,bool driveReversed,bool angleReversed,double offset,std::string name) : 
m_driveMotor{driveId,CanBus::kBusName}, 
m_angleMotor{angleId,rev::CANSparkLowLevel::MotorType::kBrushless},
m_anglingEncoder{ m_angleMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
m_name{name},
m_offset{offset}
{
    //Set Inverted
    m_driveMotor.SetInverted(driveReversed);
    m_angleMotor.SetInverted(angleReversed);

    //Convert from sensor units to radians
    m_anglingEncoder.SetPositionConversionFactor(SwerveDrive::kAngleGearRatio * M_PI * 2 / CanBus::kSparkResolution);
    m_anglingEncoder.SetVelocityConversionFactor((SwerveDrive::kAngleGearRatio * M_PI * 2) / CanBus::kSparkResolution / 60);
    
    //Convert from sensor units to meters
    ctre::phoenix6::configs::FeedbackConfigs config;
    config.WithSensorToMechanismRatio((SwerveDrive::kDriveGearRatio * SwerveDrive::kWheelDiameter * M_PI) / CanBus::KTalonResolution);
    m_driveMotor.GetConfigurator().Apply(config);
    
    //Make sure that it loop around
    m_anglingPIDController.EnableContinuousInput(-M_PI,M_PI);
    frc::SmartDashboard::PutNumber("User Control ["+m_name+"]",1);
    ResetEncoders();
    m_anglingEncoder.SetPosition(m_offset);
}

//Returns in meters
double SwerveModule::GetDrivePosition() { return m_driveMotor.GetPosition().GetValue().value(); }
//Returns in radians
double SwerveModule::GetAnglePosition() { return m_anglingEncoder.GetPosition(); }
//Returns in meters per sec
double SwerveModule::GetDriveVelocity() { return m_driveMotor.GetVelocity().GetValue().value(); }
//Returns in radians per sec
double SwerveModule::GetAngleVelocity() { return m_anglingEncoder.GetVelocity(); }

void SwerveModule::ResetEncoders() {
    m_driveMotor.SetPosition(units::angle::turn_t{ 0 });
    m_anglingEncoder.SetPosition(0);
}

#define DEBUG_SWERVE_MODULE 1

frc::SwerveModuleState SwerveModule::GetState() { return frc::SwerveModuleState{units::meters_per_second_t{ GetDriveVelocity() },frc::Rotation2d{ units::radian_t{ GetAnglePosition() } } }; }
void SwerveModule::SetState(frc::SwerveModuleState state) {
    if( std::abs(state.speed.value()) < 0.001 )
    {
        Stop();
        return;
    }
#ifdef DEBUG_SWERVE_MODULE
    frc::SmartDashboard::PutNumber("angle desire ["+m_name+"]",state.angle.Radians().value());
    frc::SmartDashboard::PutNumber("speed desire ["+m_name+"]",state.speed.value());

    frc::SmartDashboard::PutNumber("angle current ["+m_name+"]",GetState().angle.Radians().value());
    frc::SmartDashboard::PutNumber("speed current ["+m_name+"]",GetState().speed.value());

    frc::SmartDashboard::PutNumber("angle output ["+m_name+"]",state.speed.value() / SwerveDrive::kPhysicalMoveMax);
    frc::SmartDashboard::PutNumber("speed output ["+m_name+"]",m_anglingPIDController.Calculate(GetAnglePosition(), state.angle.Radians().value()));
#endif //DEBUG_SWERVE_MODULE

    state = state.Optimize(state, GetState().angle);
    m_driveMotor.Set(state.speed.value() / SwerveDrive::kPhysicalMoveMax);
    m_angleMotor.Set(m_anglingPIDController.Calculate(GetAnglePosition(), state.angle.Radians().value()));
}
void SwerveModule::Stop() {
    m_driveMotor.Set(0);
    m_angleMotor.Set(0);
}
