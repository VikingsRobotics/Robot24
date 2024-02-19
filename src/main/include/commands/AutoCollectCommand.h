#pragma once

#include "subsystems/RampSubsystem.h"
#include "subsystems/SwerveSubsystem.h"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

namespace Auto
{
    class AutoCollectCommand : public frc2::CommandHelper<frc2::Command,AutoCollectCommand>
    {
    public:
        AutoCollectCommand();
    private:
        
    };
} // namespace auto
