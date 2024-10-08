#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/sendable/SendableBuilder.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkPIDController.h>

#include <frc/DoubleSolenoid.h>
#include <frc/motorcontrol/VictorSP.h>

#include <units/angle.h>
#include <units/angular_velocity.h>

#include "Constants.h"

class RampSubsystem : public frc2::SubsystemBase 
{
public:
    RampSubsystem();

    void Stop();
    void Gather(double sweeper,double feeder);
    void Eject(double sweeper,double feeder);
#ifndef REMOVE_SOLENOID
    void SetRampDown();
    void SetRampUp();

    bool IsRampDown();
    bool IsRampUp();
#endif
    void SpoolUpLaunchers();
    void SlowSpoolUpLaunchers();
    void Fire();
    void StageNoteForLaunch();
    void StageNoteForLaunchSlow();

    void InitSendable(wpi::SendableBuilder& builder) override;

    bool retreated = true;
private:
#ifndef REMOVE_SOLENOID
    frc::DoubleSolenoid m_solenoid{Device::Internal::kPneumaticType, Device::kPneumaticBackwardId, Device::kPneumaticForwardId};
#endif
    frc::VictorSP m_sweeperMotor{Device::kSweeperMotorId};
    frc::VictorSP m_feederMotor{Device::kFeederMotorId};
    frc::VictorSP m_launcherRightMotor{Device::kTopRightMotorId};
    frc::VictorSP m_launcherLeftMotor{Device::kTopLeftMotorId};
};