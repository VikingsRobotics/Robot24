#include "commands/RampGatherCommand.h"
#include <memory>
#ifndef REMOVE_RAMP
RampGatherCommand::RampGatherCommand(RampSubsystem* const subsystem,std::function<std::pair<double,double>()> speed) : 
    m_subsystem{subsystem},m_speedFunc{std::move(speed)}
{
    AddRequirements(m_subsystem);
    
    SetName("Gather Command");
}

void RampGatherCommand::Initialize() 
{
#ifndef REMOVE_SOLENOID
    if (!m_subsystem->IsRampDown()) 
    { 
        m_subsystem->SetRampDown(); 
    }
#endif
    auto [sweeper,feeder] = m_speedFunc();
    m_subsystem->Gather(sweeper,feeder);
}

void RampGatherCommand::End(bool interrupted)
{
    m_subsystem->Stop();
}
#endif