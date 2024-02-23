#include "commands/RampLiftCommand.h"

RampLiftCommand::RampLiftCommand(RampSubsystem* const subsystem,bool direction) : m_subsystem{subsystem}, m_direction{direction}
{
    AddRequirements(m_subsystem);

    SetName("Lift Command");
}

void RampLiftCommand::Initialize()
{
    if(!m_direction) { m_subsystem->SetSolenoid(m_direction); }
}

void RampLiftCommand::End(bool interrupted)
{
    return m_subsystem->GetSolenoid();
}