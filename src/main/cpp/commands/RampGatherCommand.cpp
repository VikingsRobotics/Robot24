#include "commands/RampGatherCommand.h"

RampGatherCommand::RampGatherCommand(RampSubsystem* const subsystem) : m_subsystem{subsystem}
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
    m_subsystem->Gather();
}

void RampGatherCommand::End(bool interrupted)
{
    m_subsystem->Stop();
}