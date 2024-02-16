#include "subsystems/RampSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/FunctionalCommand.h>

RampSubsystem::RampSubsystem()
{
    SetName("Ramp Subsystem");

    frc::SmartDashboard::PutData(this);

    SetDefaultCommand(frc2::FunctionalCommand{
        [this]{
            std::array<double,4> array{0.0,0.0,0.0,0.0};
            SetMotors(array);
        },
        []{},[](bool){},[]{return false;},{this}});
    
    GetDefaultCommand()->SetName("Inactive Command");
    
    frc::SmartDashboard::PutData(GetDefaultCommand());
}

void RampSubsystem::SetMotor(Key key, double target)
{
    switch(key)
    {
        case Bottom:
            m_bottom.Set(target);
            break;
        case Loader:
            m_loader.Set(target);
            break;
        case TopRight:
            m_topRight.Set(target);
            break;
        case TopLeft:
            m_topLeft.Set(target);
        default:
            break;
    }
}

void RampSubsystem::SetMotors(std::span<double,4> targets)
{
    m_bottom.Set(targets[0]);
    m_loader.Set(targets[1]);
    m_topRight.Set(targets[2]);
    m_topLeft.Set(targets[3]);
}

void RampSubsystem::Retreat(units::turn_t position)
{
    m_loader.SetPosition(0_tr);
    ctre::phoenix6::controls::PositionVoltage control{position};
    m_loader.SetControl(control);
}

units::turn_t RampSubsystem::GetPosition()
{
    return m_loader.GetPosition().GetValue();
}

void RampSubsystem::SetVelocity(Key key,units::turns_per_second_t target)
{
    ctre::phoenix6::controls::VelocityVoltage control{target};
    switch(key)
    {
        case Bottom:
            m_bottom.SetControl(control);
            break;
        case Loader:
            m_loader.SetControl(control);
            break;
        case TopRight:
            m_topRight.SetControl(control);
            break;
        case TopLeft:
            m_topLeft.SetControl(control);
        default:
            break;
    }
}

void RampSubsystem::SetVelocities(std::span<units::turns_per_second_t,4> targets)
{
    ctre::phoenix6::controls::VelocityVoltage control{0_tps};
    m_bottom.SetControl(control.WithVelocity(targets[0]));
    m_loader.SetControl(control.WithVelocity(targets[1]));
    m_topRight.SetControl(control.WithVelocity(targets[2]));
    m_topLeft.SetControl(control.WithVelocity(targets[3]));
}

units::turns_per_second_t RampSubsystem::GetVelocity(Key key)
{
    switch(key)
    {
        case Bottom:
            return m_bottom.GetVelocity().GetValue();
        case Loader:
            return m_loader.GetVelocity().GetValue();
        case TopRight:
            return m_topRight.GetVelocity().GetValue();
        case TopLeft:
            return m_topLeft.GetVelocity().GetValue();
        default:
            return units::turns_per_second_t{std::nan("")};
    }
}

std::array<units::turns_per_second_t,4> RampSubsystem::GetVelocities()
{
    std::array<units::turns_per_second_t,4> array;
    array[0] = m_bottom.GetVelocity().GetValue();
    array[1] = m_loader.GetVelocity().GetValue();
    array[2] = m_topRight.GetVelocity().GetValue();
    array[3] = m_topLeft.GetVelocity().GetValue();
    return array;
}

void RampSubsystem::InitSendable(wpi::SendableBuilder& builder)
{
    SubsystemBase::InitSendable(builder);
    builder.AddDoubleProperty("Bottom Velocity",
        [this]{ return m_bottom.GetVelocity().GetValue().value(); },
        [this](double value){ m_bottom.SetControl(ctre::phoenix6::controls::VelocityVoltage{units::turns_per_second_t{value}}); });
    builder.AddDoubleProperty("Loader Velocity",
        [this]{ return m_loader.GetVelocity().GetValue().value(); },
        [this](double value){ m_loader.SetControl(ctre::phoenix6::controls::VelocityVoltage{units::turns_per_second_t{value}}); });
    builder.AddDoubleProperty("Top Right Velocity",
        [this]{ return m_topRight.GetVelocity().GetValue().value(); },
        [this](double value){ m_topRight.SetControl(ctre::phoenix6::controls::VelocityVoltage{units::turns_per_second_t{value}}); });
    builder.AddDoubleProperty("Top Left Velocity",
        [this]{ return m_topLeft.GetVelocity().GetValue().value(); },
        [this](double value){ m_topLeft.SetControl(ctre::phoenix6::controls::VelocityVoltage{units::turns_per_second_t{value}}); });
}