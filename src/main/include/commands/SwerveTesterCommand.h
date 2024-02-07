#pragma once
#include "subsystems/SwerveSubsystem.h"
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <frc/filter/SlewRateLimiter.h>
#include "Constants.h"

class SwerveTesterCommand : public frc2::CommandHelper<frc2::Command,SwerveTesterCommand>
{
public:

    SwerveTesterCommand(SwerveSubsystem* subsytem,bool bDirection);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool bInterupted) override;
private:
    SwerveSubsystem* m_subsystem;
    double m_speed;
    bool m_direction;
    frc::SlewRateLimiter<units::scalar> m_speeder{1 / 10_s};
};