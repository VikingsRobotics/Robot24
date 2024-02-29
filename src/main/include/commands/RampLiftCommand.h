#pragma once
#include "Constants.h"
#if !defined(REMOVE_RAMP) && !defined(REMOVE_SOLENOID)
#include "subsystems/RampSubsystem.h"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

class RampLiftCommand : public frc2::CommandHelper<frc2::Command,RampLiftCommand>
{
public:
    RampLiftCommand(RampSubsystem* const subsystem);

    void Initialize() override;
    bool IsFinished() override;
private:
    RampSubsystem* const m_subsystem;
    bool m_direction;
};
#endif