#include "commands/RampLaunchCommand.h"

#include <frc2/command/InstantCommand.h>

#include "Constants.h"

RampLaunchCommand::RampLaunchCommand(RampSubsystem* const subsystem,
    std::function<units::revolutions_per_minute_t()> rightFunc,std::function<units::revolutions_per_minute_t()> leftFunc) : 
    m_subsystem{subsystem}
{
    AddRequirements(m_subsystem);
    
    SetName("Launch Command");
    
    AddCommands(
        GetSolenoidCommand(),
        MoveLoaderDistanceCommand(Ramp::kLoaderSpeed,Ramp::kRetreatTime,true),
        SetLauncherVelocityCommand(rightFunc,leftFunc),
        MoveLoaderDistanceCommand(-Ramp::kLoaderSpeed,2 * Ramp::kRetreatTime,false)
    );
}

frc2::FunctionalCommand RampLaunchCommand::GetSolenoidCommand()
{
    return frc2::FunctionalCommand{
        [this]() {
            if(!m_subsystem->GetSolenoid()) { m_subsystem->SetSolenoid(true);}
            m_subsystem->Stop();
        },
        [](){;},
        [](bool interrupted){;},
        [this]()->bool{ return m_subsystem->GetSolenoid(); }
    };
}
frc2::ParallelCommandGroup RampLaunchCommand::MoveLoaderDistanceCommand(double speed, units::second_t time,bool retreatCheck)
{
    if(retreatCheck)
    {
        return frc2::ParallelCommandGroup{
            frc2::FunctionalCommand{ 
                [this,speed]() {
                    if(!m_subsystem->retreated) { m_subsystem->Retreat(speed); }
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
                m_subsystem->Retreat(speed);
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

frc2::FunctionalCommand RampLaunchCommand::SetLauncherVelocityCommand(
    std::function<units::revolutions_per_minute_t()> rightFunc,std::function<units::revolutions_per_minute_t()> leftFunc)
{
    auto clear = [](double a,double b,double range) { return a-b <= range && a-b >= -range; };

    return frc2::FunctionalCommand{
        [this,rightFunc,leftFunc]() {
            m_right = rightFunc();
            m_left = leftFunc();
            m_subsystem->SetLauncherVelocity(m_right.value(),m_left.value());
        },
        [](){},
        [](bool interrupted){},
        [this,clear](){
            bool vRight = clear(m_right.value(),m_subsystem->GetLauncherVelocityRight(),0.5);
            bool vLeft = clear(m_left.value(),m_subsystem->GetLauncherVelocityLeft(),0.5);
            return vRight && vLeft;
        }
    };
}