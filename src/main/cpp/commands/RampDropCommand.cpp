#include "commands/RampDropCommand.h"

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