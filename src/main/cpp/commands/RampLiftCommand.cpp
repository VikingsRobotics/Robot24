#include "commands/RampLiftCommand.h"
#if !defined(REMOVE_RAMP) && !defined(REMOVE_SOLENOID)
RampLiftCommand::RampLiftCommand(RampSubsystem* const subsystem) : m_subsystem{subsystem}
{
    AddRequirements(m_subsystem);

    SetName("Lift Command");
}

void RampLiftCommand::Initialize()
{
    m_subsystem->SetRampUp();
}

bool RampLiftCommand::IsFinished()
{
    return m_subsystem->IsRampUp();
}
#endif