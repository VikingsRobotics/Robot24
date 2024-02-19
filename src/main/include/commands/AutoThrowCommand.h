#pragma once

#include "subsystems/RampSubsystem.h"
#include "subsystems/SwerveSubsystem.h"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

namespace Auto
{
    class AutoThrowCommand : public frc2::CommandHelper<frc2::Command,AutoThrowCommand>
    {
    public:
        AutoThrowCommand();
    private:

    };
} // namespace auto
