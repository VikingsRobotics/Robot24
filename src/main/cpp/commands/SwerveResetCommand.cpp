#include "commands/SwerveResetCommand.h"

SwerveResetCommand::SwerveResetCommand(SwerveSubsystem* const subsystem) : m_subsystem{subsystem} 
{
    AddRequirements(m_subsystem);

    SetName("Reset Command");
}

void SwerveResetCommand::Initialize()
{
    m_subsystem->ZeroHeading();
}
bool SwerveResetCommand::IsFinished()
{
    return true;
}