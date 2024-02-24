#include "commands/RampLaunchCommand.h"

#include <frc2/command/InstantCommand.h>

#include "Constants.h"

RampLaunchCommand::RampLaunchCommand(RampSubsystem* const subsystem) : 
    m_subsystem{subsystem}
{
    AddRequirements(m_subsystem);
    
    SetName("Launch Command");
    
    AddCommands(
        GetSolenoidCommand(),
        MoveLoaderDistanceCommand(Ramp::kLoaderSpeed, Ramp::kRetreatTime,true),
        SetLauncherVelocityCommand(Ramp::kVelocityTime),
        MoveLoaderDistanceCommand(-Ramp::kLoaderSpeed, 2 * Ramp::kRetreatTime,false)
    );
}

frc2::FunctionalCommand RampLaunchCommand::GetSolenoidCommand()
{
    return frc2::FunctionalCommand{
        [this]() {
            if(!m_subsystem->IsRampDown()) { m_subsystem->SetRampUp();}
            m_subsystem->Stop();
        },
        [](){;},
        [](bool interrupted){;},
        [this]()->bool{ return m_subsystem->IsRampUp(); }
    };
}
frc2::ParallelCommandGroup RampLaunchCommand::MoveLoaderDistanceCommand(double speed, units::second_t time,bool retreatCheck)
{
    if(retreatCheck)
    {
        return frc2::ParallelCommandGroup{
            frc2::FunctionalCommand{ 
                [this,speed]() {
                    if(!m_subsystem->retreated) { m_subsystem->StageNoteForLaunch(); }
                },
                [](){},
                [](bool interrupted){},
                [this](){ return m_subsystem->retreated; }
            },
            frc2::SequentialCommandGroup{frc2::WaitCommand{time},frc2::InstantCommand([this](){ m_subsystem->retreated = true; })}};
    }
    return frc2::ParallelCommandGroup{
        frc2::FunctionalCommand{
            [this,speed]() {
                m_subsystem->Fire();
            },
            [](){},
            [this](bool interrupted){
                m_subsystem->Stop();
                m_subsystem->retreated = false;
            },
            [](){ return false; }
        },
        frc2::WaitCommand{time}};
}

frc2::ParallelCommandGroup RampLaunchCommand::SetLauncherVelocityCommand(units::second_t time)
{
    return frc2::ParallelCommandGroup{
        frc2::FunctionalCommand{
            [this]() {
                m_subsystem->SpoolUpLaunchers();
            },
            [](){},
            [this](bool interrupted){
                m_subsystem->Stop();
                m_subsystem->retreated = false;
            },
            [](){ return false; }
        },
        frc2::WaitCommand{time}};
}