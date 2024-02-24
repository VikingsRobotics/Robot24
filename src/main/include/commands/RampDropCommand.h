#pragma once

#include "subsystems/RampSubsystem.h"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

class RampDropCommand : public frc2::CommandHelper<frc2::Command, RampDropCommand>
{
public:
    RampDropCommand(RampSubsystem* const subsystem);

    void Initialize() override;
    bool IsFinished() override;
private:
    RampSubsystem* const m_subsystem;
};
