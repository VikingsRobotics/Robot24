#include "SwerveModule.h"
#include <math.h>

SwerveModule::SwerveModule(int driveId,int angleId,bool driveReversed,bool angleReversed) : 
m_driveMotor{driveId,CanBus::kBusName}, 
m_angleMotor{angleId,rev::CANSparkMaxLowLevel::MotorType::kBrushless}
{
    //Set Inverted
    m_driveMotor.SetInverted(driveReversed);
    m_angleMotor.SetInverted(angleReversed);

    
    //Convert from sensor units to meters
    //m_drivingEncoder = m_driveMotor.GetEncoder(); //SparkMax
    //m_drivingEncoder.SetPositionConversionFactor(CanBus::kSparkResolution * M_PI * 2);
    //m_drivingEncoder.SetVelocityConversionFactor(CanBus::kSparkResolution * M_PI * 2); 
    //Convert from sensor units to radians
    m_anglingEncoder = m_angleMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    m_anglingEncoder.SetPositionConversionFactor((SwerveDrive::kAngleGearRatio * M_PI * 2) / CanBus::kSparkResolution);
    m_anglingEncoder.SetVelocityConversionFactor((SwerveDrive::kAngleGearRatio * M_PI * 2) / CanBus::kSparkResolution);
    
    //Convert from sensor units to meters
    ctre::phoenix6::configs::FeedbackConfigs config;
    config.WithSensorToMechanismRatio((SwerveDrive::kWheelDiameter * M_PI)/ CanBus::KTalonResolution);
    m_driveMotor.GetConfigurator().Apply(config);
    
    //Make sure that it loop around
    m_anglingPIDController.EnableContinuousInput(-M_PI,M_PI);
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
frc::SwerveModuleState SwerveModule::GetState() { return frc::SwerveModuleState{units::meters_per_second_t{ GetDriveVelocity() },frc::Rotation2d{ units::radian_t{ GetAnglePosition() } } }; }
void SwerveModule::SetState(frc::SwerveModuleState state) {
    if( std::abs(state.speed.value()) < 0.001 )
    {
        Stop();
        return;
    }
    state.Optimize(state, GetState().angle);
    m_driveMotor.Set(state.speed.value() / SwerveDrive::kDriveMoveSpeedMax);
    m_angleMotor.Set(m_anglingPIDController.Calculate(GetAnglePosition(), state.angle.Radians().value()));
}
void SwerveModule::Stop() {
    m_driveMotor.Set(0);
    m_angleMotor.Set(0);
}
