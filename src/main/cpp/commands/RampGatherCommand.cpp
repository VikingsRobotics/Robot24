#include "commands/RampGatherCommand.h"

RampGatherCommand::RampGatherCommand(RampSubsystem* const subsystem) : m_subsystem{subsystem}
{
    AddRequirements(m_subsystem);
    
    SetName("Gather Command");
}

void RampGatherCommand::Initialize() 
{
    if (!m_subsystem->IsRampDown()) 
    { 
        m_subsystem->SetRampDown(); 
    }

    m_subsystem->Gather();
}

void RampGatherCommand::End(bool interrupted)
{
    m_subsystem->Stop();
}