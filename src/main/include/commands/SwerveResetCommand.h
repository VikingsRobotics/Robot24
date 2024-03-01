#pragma once
#include "Constants.h"
#ifndef REMOVE_SWERVE
#include "subsystems/SwerveSubsystem.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/filter/SlewRateLimiter.h>

#include <functional>

class SwerveResetCommand : public frc2::CommandHelper<frc2::Command,SwerveResetCommand> {
public:
    SwerveResetCommand(SwerveSubsystem* const subsystem);
    void Initialize() override;
    bool IsFinished() override;
    
private:
    //* Pointer const to swerve subsystem
    SwerveSubsystem* const m_subsystem;
};
#endif