#include "commands/RampDropCommand.h"
#if !defined(REMOVE_RAMP) && !defined(REMOVE_SOLENOID)
RampDropCommand::RampDropCommand(RampSubsystem* const subsystem) : m_subsystem{subsystem}
{
    AddRequirements(m_subsystem);

    SetName("Drop Command");
}

void RampDropCommand::Initialize()
{
    m_subsystem->SetRampDown();
}

bool RampDropCommand::IsFinished()
{
    return m_subsystem->IsRampDown();
}
#endif