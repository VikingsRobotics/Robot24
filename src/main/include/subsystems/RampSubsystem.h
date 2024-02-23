#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/sendable/SendableBuilder.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkPIDController.h>

#include <frc/Solenoid.h>

#include <units/angle.h>
#include <units/angular_velocity.h>

#include "Constants.h"

class RampSubsystem : public frc2::SubsystemBase 
{
public:
    RampSubsystem();

    void Stop();
    
    void Gather(double bottom,double loader);

    void SetLauncherSpeed(double right,double left);
    void SetLauncherVelocity(double targetRight,double targetLeft);

    double GetLauncherVelocityRight();
    double GetLauncherVelocityLeft();

    void Retreat(double speed);

    bool GetSolenoid();
    void SetSolenoid(bool on);

    void InitSendable(wpi::SendableBuilder& builder) override;
public:
    bool retreated;
private:
    frc::Solenoid m_solenoid{Device::Internal::kPneumaticType,Device::kPneumaticId};
    rev::CANSparkMax m_bottom{Device::kBottomMotorId,Device::Internal::kSparkMotorType};
    rev::CANSparkMax m_loader{Device::kLoaderMotorId,Device::Internal::kSparkMotorType};
    rev::CANSparkMax m_launcherRight{Device::kTopRightMotorId,Device::Internal::kSparkMotorType};
    rev::CANSparkMax m_launcherLeft{Device::kTopLeftMotorId,Device::Internal::kSparkMotorType};
    rev::SparkPIDController m_pidRight{m_launcherRight.GetPIDController()};
    rev::SparkPIDController m_pidLeft{m_launcherLeft.GetPIDController()};
    rev::SparkRelativeEncoder m_encoderRight{m_launcherRight.GetEncoder(Device::Internal::kSparkRelEncoderType,Device::Internal::kSparkResolution)};
    rev::SparkRelativeEncoder m_encoderLeft{m_launcherLeft.GetEncoder(Device::Internal::kSparkRelEncoderType,Device::Internal::kSparkResolution)};
};