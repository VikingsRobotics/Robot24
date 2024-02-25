#pragma once

#include "subsystems/RampSubsystem.h"

#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/CommandHelper.h>
#include <functional>

#include <units/angular_velocity.h>

class RampLaunchCommand : public frc2::CommandHelper<frc2::SequentialCommandGroup,RampLaunchCommand>
{
public:
    RampLaunchCommand(RampSubsystem* const subsystem);
private:
#ifndef REMOVE_SOLENOID
    frc2::FunctionalCommand GetSolenoidCommand();
#endif
    frc2::ParallelCommandGroup MoveLoaderDistanceCommand(double speed, units::second_t time,bool retreatCheck);
    frc2::ParallelCommandGroup SetLauncherVelocityCommand(units::second_t time);
    RampSubsystem* const m_subsystem = nullptr;
    units::turns_per_second_t m_right;
    units::turns_per_second_t m_left;
};