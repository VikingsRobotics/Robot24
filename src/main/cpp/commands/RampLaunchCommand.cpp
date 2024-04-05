#include "commands/RampLaunchCommand.h"

#include <frc2/command/InstantCommand.h>

#include "Constants.h"
#ifndef REMOVE_RAMP
RampLaunchCommand::RampLaunchCommand(RampSubsystem* const subsystem,bool disableSolenoid,bool direction) : 
    m_subsystem{subsystem}
{
    AddRequirements(m_subsystem);
    
    SetName("Launch Command");
    
#ifndef REMOVE_SOLENOID
    if(!disableSolenoid)
    {
        AddCommands(
            GetSolenoidCommand(direction)
        );
    }
#endif
    AddCommands(
        MoveLoaderDistanceCommand(Ramp::kRetreatTime,true),
        SetLauncherVelocityCommand(Ramp::kVelocityTime),
        MoveLoaderDistanceCommand(Ramp::kLaunchTime,false)
    );
}
#ifndef REMOVE_SOLENOID
frc2::FunctionalCommand RampLaunchCommand::GetSolenoidCommand(bool direction)
{
    if(direction)
    {
        return frc2::FunctionalCommand{
            [rampsub = this->m_subsystem]() {
                if(rampsub->IsRampDown()) { rampsub->SetRampUp();}
                rampsub->Stop();
            },
            [](){;},
            [](bool interrupted){;},
            [rampsub = this->m_subsystem]()->bool{ return rampsub->IsRampUp(); },
            {m_subsystem}
        };
    }
    return frc2::FunctionalCommand{
        [rampsub = this->m_subsystem]() {
            if(rampsub->IsRampUp()) { rampsub->SetRampDown();}
            rampsub->Stop();
        },
        [](){;},
        [](bool interrupted){;},
        [rampsub = this->m_subsystem]()->bool{ return rampsub->IsRampDown(); },
        {m_subsystem}
    };
}
#endif
frc2::ParallelRaceGroup RampLaunchCommand::MoveLoaderDistanceCommand(units::second_t time,bool retreatCheck)
{
    if(retreatCheck)
    {
        return frc2::ParallelRaceGroup{
            frc2::FunctionalCommand{ 
                [rampsub = this->m_subsystem]() {
                    if(!rampsub->retreated && rampsub->IsRampUp()) { rampsub->StageNoteForLaunch(); }
                    else if(!rampsub->retreated && rampsub->IsRampDown()) { rampsub->StageNoteForLaunchSlow(); }
                },
                [](){},
                [rampsub = this->m_subsystem](bool interrupted){
                    rampsub->Stop();
                },
                [rampsub = this->m_subsystem](){ return rampsub->retreated; },
                {m_subsystem}
            },
            frc2::SequentialCommandGroup{frc2::WaitCommand{time},frc2::InstantCommand([rampsub = this->m_subsystem](){ rampsub->retreated = true; })}};
    }
    return frc2::ParallelRaceGroup{
        frc2::FunctionalCommand{
            [rampsub = this->m_subsystem]() {
                rampsub->Fire();
            },
            [](){},
            [rampsub = this->m_subsystem](bool interrupted){
                rampsub->Stop();
                rampsub->retreated = false;
            },
            [](){ return false; },
            {m_subsystem}
        },
        frc2::WaitCommand{time}};
}

frc2::ParallelRaceGroup RampLaunchCommand::SetLauncherVelocityCommand(units::second_t time)
{
    return frc2::ParallelRaceGroup{
        frc2::FunctionalCommand{
            [rampsub = this->m_subsystem]() {
                if(rampsub->IsRampDown())
                {
                    rampsub->SlowSpoolUpLaunchers();
                }
                else
                {
                    rampsub->SpoolUpLaunchers();
                }
            },
            [](){},
            [rampsub = this->m_subsystem](bool interrupted){
                rampsub->retreated = false;
            },
            [](){ return false; },
            {m_subsystem}
        },
        frc2::WaitCommand{time}};
}
#endif