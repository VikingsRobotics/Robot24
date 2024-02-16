#include "subsystems/SwerveModule.h"
#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>


SwerveModule::SwerveModule(const int drivingCANId, const int turningCANId,
                                 const double chassisAngularOffset)
    : m_drivingTalonFx(drivingCANId, Device::kBusName),
      m_turningSparkMax(turningCANId, Device::Internal::kSparkMotorType),
      m_turningAbsoluteEncoder(m_turningSparkMax.GetAbsoluteEncoder(Device::Internal::kSparkAbsEncoderType)),
      m_turningPIDController(m_turningSparkMax.GetPIDController()),
      m_chassisAngularOffset(chassisAngularOffset)
{
    // Spark Max Settings
    
    // Set default settings
    m_turningSparkMax.RestoreFactoryDefaults();
    // Make sure that position is radians and not rotations
    m_turningAbsoluteEncoder.SetPositionConversionFactor(Swerve::Mechanism::kAngleGearRatio);
    m_turningAbsoluteEncoder.SetVelocityConversionFactor(Swerve::Mechanism::kAngleGearRatio / 60);
    // Make sure that the encoder follows the direction of motors
    m_turningAbsoluteEncoder.SetInverted(Device::Internal::kInvertEncoder);
    // Make the PID continuous 
    m_turningPIDController.SetPositionPIDWrappingEnabled(true);
    m_turningPIDController.SetPositionPIDWrappingMinInput(0);
    m_turningPIDController.SetPositionPIDWrappingMaxInput(std::numbers::pi * 2);
    // Make sure motor drives according to the encoder
    m_turningPIDController.SetFeedbackDevice(m_turningAbsoluteEncoder);
    // Set the PIDF terms and output range
    m_turningPIDController.SetP(Swerve::System::kTurningPControl);
    m_turningPIDController.SetI(0);
    m_turningPIDController.SetD(0);
    m_turningPIDController.SetFF(0);
    m_turningPIDController.SetOutputRange(-1, 1);
    // Make sure it doesn't coast or exceed 20 A
    m_turningSparkMax.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_turningSparkMax.SetSmartCurrentLimit(20);
    // Burn into flash so if reset, it remembers
    m_turningSparkMax.BurnFlash();

    // Talon FX Settings

    // Create a default settings config object
    ctre::phoenix6::configs::TalonFXConfiguration config{};
    // Sets the voltages to their nominal voltage
    config.Voltage.WithPeakForwardVoltage(12).WithPeakReverseVoltage(-12);
    // When doing control loop, makes sure we are talk about mechanism and not motor
    config.Feedback.WithSensorToMechanismRatio(Swerve::Mechanism::kDriveGearRatio); //Possible it does nothing, if so, just use DriveSpeedToTurns constant
    // Set the PIDF terms
    config.Slot0.WithKS(Swerve::Mechanism::kStaticVoltage.value()).WithKV(Swerve::Mechanism::kVelocityVoltage.value()).WithKP(Swerve::System::kVelocityPControl);
    // Make sure it doesn't coast
    config.MotorOutput.WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    // Apply the settings
    m_drivingTalonFx.GetConfigurator().Apply(config);

    // Sets the state as current rotations
    m_previousState.angle = frc::Rotation2d(units::radian_t{m_turningAbsoluteEncoder.GetPosition()});
    // Make sure the motor's encoders are set to 0
    ResetEncoders();
}    

void SwerveModule::ResetEncoders()
{
    // Reset the encoder to 0
    m_drivingTalonFx.SetPosition(units::angle::turn_t{ 0 });
}

frc::SwerveModuleState SwerveModule::GetState() { return frc::SwerveModuleState{units::meters_per_second_t{ m_drivingTalonFx.GetVelocity().GetValue().value() * Swerve::Mechanism::kTurnsToDriveSpeed },frc::Rotation2d{ units::radian_t{ m_turningAbsoluteEncoder.GetPosition() } - m_chassisAngularOffset } }; }

frc::SwerveModulePosition SwerveModule::GetPosition() { return frc::SwerveModulePosition{units::meter_t{ m_drivingTalonFx.GetPosition().GetValue().value() * Swerve::Mechanism::kTurnsToDriveSpeed},frc::Rotation2d{ units::radian_t{ m_turningAbsoluteEncoder.GetPosition() } - m_chassisAngularOffset } }; }

void SwerveModule::SetState(frc::SwerveModuleState desiredState) 
{
    // If we are not moving, we don't reset back to origin
    if( std::abs(desiredState.speed.value()) < 0.00001 )
    {
        Stop();
        return;
    }
    // Make sure it is all aligned correctly, trivially assignable
    frc::SwerveModuleState correctedDesiredState{};
    correctedDesiredState.speed = desiredState.speed;
    correctedDesiredState.angle =
        desiredState.angle +
        frc::Rotation2d(m_chassisAngularOffset);
    // Make sure we are not rotating too much
    frc::SwerveModuleState optimizedDesiredState{frc::SwerveModuleState::Optimize(
        correctedDesiredState, frc::Rotation2d(units::radian_t{
                                    m_turningAbsoluteEncoder.GetPosition()}))};

    // Sets the velocity of the driving motors
    m_drivingTalonFx.SetControl(ctre::phoenix6::controls::VelocityVoltage{units::turns_per_second_t(
        optimizedDesiredState.speed.value() / Swerve::Mechanism::kWheelCircumference.value()
        /*if doesn't work, uncomment optimizedDesiredState.speed.value() * SwerveDrive::kDriveSpeedToTurns */)}.WithSlot(0));
    // Sets the position of the turning motors
    m_turningPIDController.SetReference(
        optimizedDesiredState.angle.Radians().value(),
        rev::CANSparkMax::ControlType::kPosition);
    // Remember the state for debugging
    m_previousState = desiredState;
}
void SwerveModule::Stop() 
{
    // Just stop movement
    m_drivingTalonFx.Set(0);
    m_turningSparkMax.Set(0);
}
