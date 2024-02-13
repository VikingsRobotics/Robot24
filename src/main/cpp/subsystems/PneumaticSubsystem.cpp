#include "subsystems/PneumaticSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

frc::Compressor PneumaticSubsystem::m_compressor{Device::Internal::kPneumaticType};

PneumaticSubsystem::PneumaticSubsystem(int channel) : m_solenoid{Device::Internal::kPneumaticType,channel}
{
    //Set our own name
    SetName("Pneumatics Subsystem");
    //Send to the dashboard
    frc::SmartDashboard::PutData(this);
}

bool PneumaticSubsystem::Toggle()
{
    //Make sure we are at max pressure
    if(IsPressurized())
    {   
        //Toggles the solenoid
        m_solenoid.Toggle();
        //Inform that the solenoid was triggered
        return true;
    }
    //Inform that it not at max pressure
    return false;
}

bool PneumaticSubsystem::Set(bool on)
{
    //Make sure we are at max pressure
    if(IsPressurized())
    {
        //Set the solenoid
        m_solenoid.Set(on);
        //Inform that the solenoid was triggered
        return true;
    }
    //Inform that it not at max pressure
    return false;
}

bool PneumaticSubsystem::Get() { return m_solenoid.Get(); }
bool PneumaticSubsystem::IsPressurized() { return m_compressor.GetPressureSwitchValue(); }