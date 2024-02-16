#include "commands/RampLaunchCommand.h"

#include <frc2/command/Commands.h>

#include "Constants.h"

RampLaunchCommand::RampLaunchCommand(RampSubsystem* const subsystem,
    units::turns_per_second_t right,units::turns_per_second_t left) : m_subsystem{subsystem},
    m_right{right},m_left{left}
{
    AddRequirements(m_subsystem);
    
    SetName("Gather Command");
}

void RampLaunchCommand::Initialize() 
{
    std::array<double,4> array{0.0,0.0,0.0,0.0};
    m_subsystem->SetMotors(array);
    m_subsystem->Retreat(Device::Internal::kRetreatDistance);
    m_state = 0;
}

void RampLaunchCommand::Execute()
{
    auto clearTps = [](units::turns_per_second_t a,units::turns_per_second_t b) { return (a-b).value() < 0.1 && (a-b).value() > -0.1; };
    auto clear = [](units::turn_t a,units::turn_t b) { return (a-b).value() < 0.1 && (a-b).value() > -0.1; };

    std::array<units::turns_per_second_t,4> varray{0_tps,0_tps,m_right,m_left};
    if((~m_state & 0x1) && clear(Device::Internal::kRetreatDistance,m_subsystem->GetPosition()))
    {
        m_state |= 0x1;
        m_subsystem->SetVelocities(varray);
    }
    if((~m_state & 0x1)) { return; }
    varray = m_subsystem->GetVelocities();
    if((~m_state & 0x2) && clearTps(m_right,m_subsystem->GetVelocity(RampSubsystem::TopRight)) &&
        clearTps(m_left,m_subsystem->GetVelocity(RampSubsystem::TopLeft)))
    {
        m_state |= 0x2;
        m_subsystem->Retreat(-2 * Device::Internal::kRetreatDistance);
    }
    if((~m_state & 0x2)) { return; }
    if((~m_state & 0x4) && clear(-2 * Device::Internal::kRetreatDistance,m_subsystem->GetPosition())) { m_state |= 0x4; }
}

bool RampLaunchCommand::IsFinished()
{
    return m_state & 0x4;
}

void RampLaunchCommand::End(bool interrupted)
{
    std::array<double,4> array{0.0,0.0,0.0,0.0};
    m_subsystem->SetMotors(array);
}