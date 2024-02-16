#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include <units/angle.h>
#include <units/angular_velocity.h>

#include "Constants.h"

class RampSubsystem : public frc2::SubsystemBase 
{
public:
    enum Key : size_t
    {
        Bottom = 0,
        Loader = 1,
        TopRight = 2,
        TopLeft = 3
    };
    RampSubsystem();

    void SetMotor(Key key,double target);
    void SetMotors(std::span<double,4> targets);

    void Retreat(units::turn_t position);
    units::turn_t GetPosition();

    void SetVelocity(Key key,units::turns_per_second_t target);
    void SetVelocities(std::span<units::turns_per_second_t,4> targets);

    units::turns_per_second_t GetVelocity(Key key);
    std::array<units::turns_per_second_t,4> GetVelocities();

    void InitSendable(wpi::SendableBuilder& builder);
private:
    ctre::phoenix6::hardware::TalonFX m_bottom{Device::kBottomMotorId,Device::kBusName};
    ctre::phoenix6::hardware::TalonFX m_loader{Device::kBottomMotorId,Device::kBusName};
    ctre::phoenix6::hardware::TalonFX m_topRight{Device::kBottomMotorId,Device::kBusName};
    ctre::phoenix6::hardware::TalonFX m_topLeft{Device::kBottomMotorId,Device::kBusName};
};