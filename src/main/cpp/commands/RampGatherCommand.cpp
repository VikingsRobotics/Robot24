#include "commands/RampGatherCommand.h"

RampGatherCommand::RampGatherCommand(RampSubsystem* const subsystem) : m_subsystem{subsystem}
{
    AddRequirements(m_subsystem);
    
    SetName("Gather Command");
}

void RampGatherCommand::Initialize() 
{
    if(m_subsystem->GetSolenoid()) { m_subsystem->SetSolenoid(false); }
    m_subsystem->Gather(0.7,0.5);
}

void RampGatherCommand::End(bool interrupted)
{
    m_subsystem->Stop();
}