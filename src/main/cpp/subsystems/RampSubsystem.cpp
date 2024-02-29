#include "subsystems/RampSubsystem.h"

#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/RunCommand.h>

RampSubsystem::RampSubsystem()
{
    SetName("Ramp Subsystem");
    m_sweeperMotor.SetInverted(true);
    m_launcherLeftMotor.SetInverted(false);
    m_launcherRightMotor.SetInverted(true);

    SetDefaultCommand(frc2::RunCommand{
        [this]{
            m_sweeperMotor.Set(0);
            m_feederMotor.Set(0);
            m_launcherRightMotor.Set(0);
            m_launcherLeftMotor.Set(0);
        }, {this}});
    
    GetDefaultCommand()->SetName("Inactive Command");
    frc::SmartDashboard::PutData(GetDefaultCommand());
    frc::SmartDashboard::PutData(this);
#ifndef REMOVE_SOLENOID
    frc::SmartDashboard::PutData("Ramp Solenoid", &m_solenoid);
#endif
}

void RampSubsystem::Stop()
{
    m_sweeperMotor.Set(0);
    m_feederMotor.Set(0);
    m_launcherRightMotor.Set(0);
    m_launcherLeftMotor.Set(0);
}

void RampSubsystem::Gather()
{
    m_sweeperMotor.Set(.75);
    m_feederMotor.Set(.7);
}

void RampSubsystem::Eject()
{
    m_sweeperMotor.Set(-.7);
    m_feederMotor.Set(-.5);
}

void RampSubsystem::SpoolUpLaunchers()
{
    m_launcherLeftMotor.Set(1);
    m_launcherRightMotor.Set(1);
}

void RampSubsystem::SlowSpoolUpLaunchers()
{
    m_launcherLeftMotor.Set(0.4);
    m_launcherRightMotor.Set(0.4);
}

/// @brief This backs the note down so that it's not engaged with the launchers
void RampSubsystem::StageNoteForLaunch()
{
    m_sweeperMotor.Set(0);
    m_feederMotor.Set(-.3);
}

/// @brief This pushes the note into the launchers. This is what actually sends the note flying.
void RampSubsystem::Fire()
{
    m_feederMotor.Set(.4);
}
#ifndef REMOVE_SOLENOID
bool RampSubsystem::IsRampDown() 
{ 
    return m_solenoid.Get() == frc::DoubleSolenoid::Value::kReverse; 
}

bool RampSubsystem::IsRampUp()
{
    return m_solenoid.Get() == frc::DoubleSolenoid::Value::kForward; 
}

void RampSubsystem::SetRampDown()
{
    m_solenoid.Set(frc::DoubleSolenoid::Value::kReverse); 
}

void RampSubsystem::SetRampUp()
{
    m_solenoid.Set(frc::DoubleSolenoid::Value::kForward);
}
#endif
void RampSubsystem::InitSendable(wpi::SendableBuilder& builder)
{
    SubsystemBase::InitSendable(builder);
    builder.AddBooleanProperty("Sweeper Motor Active", [this]{ return m_sweeperMotor.Get() != 0; }, nullptr);
    builder.AddBooleanProperty("Feeder Motor Active", [this]{ return m_feederMotor.Get() != 0; }, nullptr);
    builder.AddDoubleProperty("Launcher Right Velocity",
        [this]{ return m_launcherRightMotor.Get(); },
        [this](double value){ m_launcherRightMotor.Set(value); });
    builder.AddDoubleProperty("Launcher Left Velocity",
        [this]{ return m_launcherLeftMotor.Get(); },
        [this](double value){ m_launcherLeftMotor.Set(value); });
    builder.AddBooleanProperty("Retreated", [this]{ return retreated; }, [this](bool value){ retreated = value; });
}