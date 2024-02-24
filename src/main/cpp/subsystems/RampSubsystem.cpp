#include "subsystems/RampSubsystem.h"

#include <frc2/command/FunctionalCommand.h>

RampSubsystem::RampSubsystem()
{
    SetName("Ramp Subsystem");

    frc::SmartDashboard::PutData(this);

    SetDefaultCommand(frc2::FunctionalCommand{
        [this]{
            m_bottom.Set(0);
            m_loader.Set(0);
            m_launcherRight.Set(0);
            m_launcherLeft.Set(0);
        },
        []{},[](bool){},[]{return false;},{this}});
    
    GetDefaultCommand()->SetName("Inactive Command");
    
    frc::SmartDashboard::PutData(GetDefaultCommand());

    frc::SmartDashboard::PutData("Ramp Solenoid",&m_solenoid);

    m_pidLeft.SetP(0.5);
    m_pidLeft.SetI(0);
    m_pidLeft.SetD(0);
    m_pidLeft.SetFF(1/5676);

    m_pidRight.SetP(0.5);
    m_pidRight.SetI(0);
    m_pidRight.SetD(0);
    m_pidRight.SetFF(1/5676);
}

void RampSubsystem::Stop()
{
    m_bottom.Set(0);
    m_loader.Set(0);
    m_launcherRight.Set(0);
    m_launcherLeft.Set(0);
}
    
void RampSubsystem::Gather(double bottom,double loader)
{
    m_bottom.Set(bottom);
    m_loader.Set(loader);
}

void RampSubsystem::SetLauncherSpeed(double right,double left)
{
    m_launcherRight.Set(right);
    m_launcherLeft.Set(left);
}
    
void RampSubsystem::SetLauncherVelocity(double targetRight,double targetLeft)
{
    m_pidRight.SetReference(targetRight,rev::CANSparkBase::ControlType::kVelocity);
    m_pidLeft.SetReference(targetLeft,rev::CANSparkBase::ControlType::kVelocity);
}

double RampSubsystem::GetLauncherVelocityRight()
{
    return m_encoderRight.GetVelocity();
}
double RampSubsystem::GetLauncherVelocityLeft()
{
    return m_encoderLeft.GetVelocity();
}

void RampSubsystem::Retreat(double speed)
{
    m_loader.Set(speed);
}

bool RampSubsystem::GetSolenoid() { return m_solenoid.Get() == frc::DoubleSolenoid::Value::kReverse; }

void RampSubsystem::SetSolenoid(bool on) { 
    if(on)
    { m_solenoid.Set(frc::DoubleSolenoid::Value::kForward); } 
    else
    { m_solenoid.Set(frc::DoubleSolenoid::Value::kReverse); }
}

void RampSubsystem::InitSendable(wpi::SendableBuilder& builder)
{
    SubsystemBase::InitSendable(builder);
    builder.AddBooleanProperty("Bottom Active",[this]{ return m_bottom.Get() != 0; },nullptr);
    builder.AddBooleanProperty("Loader Active",[this]{ return m_loader.Get() != 0; },nullptr);
    builder.AddDoubleProperty("Top Right Velocity",
        [this]{ return m_encoderRight.GetVelocity()/60; },
        [this](double value){ m_pidRight.SetReference(value/60,rev::CANSparkBase::ControlType::kVelocity); });
    builder.AddDoubleProperty("Top Left Velocity",
        [this]{ return m_encoderLeft.GetVelocity()/60; },
        [this](double value){ m_pidLeft.SetReference(value/60,rev::CANSparkBase::ControlType::kVelocity); });
    builder.AddBooleanProperty("Retreated",[this]{ return retreated; }, [this](bool value){ retreated = value; });
}