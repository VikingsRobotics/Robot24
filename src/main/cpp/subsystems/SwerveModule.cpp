#include "subsystems/SwerveModule.h"
#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>


SwerveModule::SwerveModule(const int drivingCANId, const int turningCANId,
                                 const double chassisAngularOffset)
    : m_drivingTalonFx(drivingCANId, CanBus::kBusName),
      m_turningSparkMax(turningCANId, CanBus::kSparkMotorType),
      m_turningAbsoluteEncoder(m_turningSparkMax.GetAbsoluteEncoder(CanBus::kSparkAbsEncoderType)),
      m_turningPIDController(m_turningSparkMax.GetPIDController())
{
    //m_drivingSparkMax.RestoreFactoryDefaults();
    m_turningSparkMax.RestoreFactoryDefaults();

    //m_drivingEncoder.SetPositionConversionFactor(kDrivingEncoderPositionFactor);
    //m_drivingEncoder.SetVelocityConversionFactor(kDrivingEncoderVelocityFactor);

    m_turningAbsoluteEncoder.SetPositionConversionFactor(SwerveDrive::kAngleGearRatio);
    m_turningAbsoluteEncoder.SetVelocityConversionFactor(SwerveDrive::kAngleGearRatio / 60);

    m_turningAbsoluteEncoder.SetInverted(CanBus::kInvertEncoder);

    m_turningPIDController.SetPositionPIDWrappingEnabled(true);
    m_turningPIDController.SetPositionPIDWrappingMinInput(0);
    m_turningPIDController.SetPositionPIDWrappingMaxInput(std::numbers::pi * 2);
            
    m_turningPIDController.SetFeedbackDevice(m_turningAbsoluteEncoder);

    //m_drivingPIDController.SetP(kDrivingP);
    //m_drivingPIDController.SetI(kDrivingI);
    //m_drivingPIDController.SetD(kDrivingD);
    //m_drivingPIDController.SetFF(kDrivingFF);
    //m_drivingPIDController.SetOutputRange(kDrivingMinOutput, kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and
    // you may need to tune them for your own robot!
    m_turningPIDController.SetP(SwerveDrive::kTurningPControl);
    m_turningPIDController.SetI(0);
    m_turningPIDController.SetD(0);
    m_turningPIDController.SetFF(0);
    m_turningPIDController.SetOutputRange(-1, 1);

    //m_drivingSparkMax.SetIdleMode(kDrivingMotorIdleMode);
    m_turningSparkMax.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    //m_drivingSparkMax.SetSmartCurrentLimit(kDrivingMotorCurrentLimit.value());
    m_turningSparkMax.SetSmartCurrentLimit(20);

    //m_drivingSparkMax.BurnFlash();
    m_turningSparkMax.BurnFlash();

    ctre::phoenix6::configs::TalonFXConfiguration config{};

    config.Feedback.WithSensorToMechanismRatio(SwerveDrive::kDriveGearRatio);
    config.Slot0.WithKS(SwerveDrive::kStaticVoltage.value()).WithKV(SwerveDrive::kVelocityVoltage.value()).WithKP(SwerveDrive::kVelocityPControl);

    m_drivingTalonFx.GetConfigurator().Apply(config);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle =
        frc::Rotation2d(units::radian_t{m_turningAbsoluteEncoder.GetPosition()});
    ResetEncoders();
}    

void SwerveModule::ResetEncoders() {
    m_drivingTalonFx.SetPosition(units::angle::turn_t{ 0 });
}
frc::SwerveModuleState SwerveModule::GetState() { return frc::SwerveModuleState{units::meters_per_second_t{ m_drivingTalonFx.GetVelocity().GetValue().value() },frc::Rotation2d{ units::radian_t{ m_turningAbsoluteEncoder.GetPosition() - m_chassisAngularOffset } } }; }
frc::SwerveModulePosition SwerveModule::GetPosition() { return frc::SwerveModulePosition{units::meter_t{ m_drivingTalonFx.GetPosition().GetValue().value() },frc::Rotation2d{ units::radian_t{ m_turningAbsoluteEncoder.GetPosition() - m_chassisAngularOffset } } }; }
void SwerveModule::SetState(frc::SwerveModuleState desiredState) {
    if( std::abs(desiredState.speed.value()) < 0.00001 )
    {
        Stop();
        return;
    }
    frc::SwerveModuleState correctedDesiredState{};
    correctedDesiredState.speed = desiredState.speed;
    correctedDesiredState.angle =
        desiredState.angle +
        frc::Rotation2d(units::radian_t{m_chassisAngularOffset});

    frc::SwerveModuleState optimizedDesiredState{frc::SwerveModuleState::Optimize(
        correctedDesiredState, frc::Rotation2d(units::radian_t{
                                    m_turningAbsoluteEncoder.GetPosition()}))};

    frc::SmartDashboard::PutNumber("["+std::to_string(m_drivingTalonFx.GetDeviceID())+"] rot out",desiredState.speed.value() / SwerveDrive::kWheelCircumeter.value());
    m_drivingTalonFx.SetControl(ctre::phoenix6::controls::VelocityVoltage{units::turns_per_second_t(desiredState.speed.value() / SwerveDrive::kWheelCircumeter.value())}.WithSlot(0));
    frc::SmartDashboard::PutNumber("["+std::to_string(m_drivingTalonFx.GetDeviceID())+"] volt out",m_drivingTalonFx.GetMotorVoltage().GetValue().value());
    m_turningPIDController.SetReference(
        optimizedDesiredState.angle.Radians().value(),
        rev::CANSparkMax::ControlType::kPosition);
    m_desiredState = desiredState;
}
void SwerveModule::Stop() {
    m_drivingTalonFx.Set(0);
    m_turningSparkMax.Set(0);
}
