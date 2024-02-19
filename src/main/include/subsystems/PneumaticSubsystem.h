#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/Solenoid.h>
#include <frc/Compressor.h>

#include "Constants.h"

class PneumaticSubsystem : public frc2::SubsystemBase 
{
public:
    /**
     * Constructs a pneumatic interface
     * 
     * @param channel id of the channel that the pneumatic solenoid is under
    */
    PneumaticSubsystem();
    /** 
     * Toogles the solenoid if the compressor is full
     * 
     * @return if solenoid was triggered
    */
    bool Toggle();
    /** 
     * Set the solenoid if the compressor is full
     * 
     * @param on direction of solenoid
     * 
     * @return if solenoid was triggered
    */ 
    bool Set(bool on);
    //* @return the direction of the solenoid
    bool Get();
    //* @return whether the compressor is full 
    bool IsPressurized();
private:
    //* The solenoid with the id
    frc::Solenoid m_solenoid{Device::Internal::kPneumaticType,Device::kPneumaticId};
    //* Universal compressor 
    frc::Compressor m_compressor{Device::Internal::kPneumaticType};
};