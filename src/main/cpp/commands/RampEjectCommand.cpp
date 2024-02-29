#include "commands/RampEjectCommand.h"
#include <memory>
#ifndef REMOVE_RAMP
RampEjectCommand::RampEjectCommand(RampSubsystem* const subsystem,std::function<std::pair<double,double>()> speed) : 
    m_subsystem{subsystem},m_speedFunc{std::move(speed)}
{
    AddRequirements(m_subsystem);
    
    SetName("Eject Command");
}

void RampEjectCommand::Initialize() 
{
#ifndef REMOVE_SOLENOID
    if (!m_subsystem->IsRampDown()) 
    { 
        m_subsystem->SetRampDown(); 
    }
#endif
    auto [sweeper,feeder] = m_speedFunc();
    m_subsystem->Eject(sweeper,feeder);
}

void RampEjectCommand::End(bool interrupted)
{
    m_subsystem->Stop();
}
#endif