#pragma once
#include "Constants.h"
#ifndef REMOVE_RAMP
#include "subsystems/RampSubsystem.h"

#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/CommandHelper.h>
#include <functional>

#include <units/angular_velocity.h>

class RampLaunchCommand : public frc2::CommandHelper<frc2::SequentialCommandGroup,RampLaunchCommand>
{
public:
    RampLaunchCommand(RampSubsystem* const subsystem,bool disableSolenoid,bool direction = true);
private:
#ifndef REMOVE_SOLENOID
    /**
     * @param direction ramp is set to up when true, down when false
    */
    frc2::FunctionalCommand GetSolenoidCommand(bool direction);
#endif
    frc2::ParallelRaceGroup MoveLoaderDistanceCommand(units::second_t time,bool retreatCheck);
    frc2::ParallelRaceGroup SetLauncherVelocityCommand(units::second_t time);
    RampSubsystem* const m_subsystem;
};
#endif