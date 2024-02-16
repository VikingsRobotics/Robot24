
#pragma once

#include "subsystems/RampSubsystem.h"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include <units/angular_velocity.h>

class RampLaunchCommand : public frc2::CommandHelper<frc2::Command,RampLaunchCommand>
{
public:
    RampLaunchCommand(RampSubsystem* const subsystem,units::turns_per_second_t right,units::turns_per_second_t left);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;
private:
    RampSubsystem* const m_subsystem;
    uint8_t m_state;
    units::turns_per_second_t m_right;
    units::turns_per_second_t m_left;
};