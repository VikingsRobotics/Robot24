#pragma once
#include "Constants.h"
#ifndef REMOVE_RAMP
#include "subsystems/RampSubsystem.h"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include <functional>
#include <utility>

class RampEjectCommand : public frc2::CommandHelper<frc2::Command,RampEjectCommand>
{
public:
    RampEjectCommand(RampSubsystem* const subsystem,std::function<std::pair<double,double>()> speed = []->std::pair<double,double>{return {0.85,0.7};});

    void Initialize() override;
    void End(bool interrupted) override;
private:
    RampSubsystem* const m_subsystem;
    std::function<std::pair<double,double>()> m_speedFunc;
};
#endif