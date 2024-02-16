#include "commands/RampGatherCommand.h"

RampGatherCommand::RampGatherCommand(RampSubsystem* const subsystem) : m_subsystem{subsystem}
{
    AddRequirements(m_subsystem);
    
    SetName("Gather Command");
}

void RampGatherCommand::Initialize() 
{
    std::array<double,4> array{0.0,0.0,0.0,0.0};
    m_subsystem->SetMotors(array);
}

void RampGatherCommand::Execute()
{
    std::array<double,4> array{0.7,0.5,0.0,0.0};
    m_subsystem->SetMotors(array);
}

void RampGatherCommand::End(bool interrupted)
{
    std::array<double,4> array{0.0,0.0,0.0,0.0};
    m_subsystem->SetMotors(array);
}