#pragma once
#include "Constants.h"
#ifndef REMOVE_RAMP
#include "subsystems/RampSubsystem.h"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

class RampGatherCommand : public frc2::CommandHelper<frc2::Command,RampGatherCommand>
{
public:
    RampGatherCommand(RampSubsystem* const subsystem);

    void Initialize() override;
    void End(bool interrupted) override;
private:
    RampSubsystem* const m_subsystem;
};
#endif