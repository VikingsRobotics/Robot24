#pragma once

#include "subsystems/SwerveSubsystem.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/filter/SlewRateLimiter.h>

#include <functional>

class SwerveDriveCommand : public frc2::CommandHelper<frc2::Command,SwerveDriveCommand> {
public:
    SwerveDriveCommand(SwerveSubsystem* subsystem,std::function<double(void)> xSpdFunc,
    std::function<double(void)> ySpdFunc,std::function<double(void)> aSpdFunc);

    void Execute() override;
    void End(bool interrupted) override;
    
private:
    SwerveSubsystem* m_subsystem;

    std::function<double(void)> m_xSpdFunc;
    std::function<double(void)> m_ySpdFunc;
    std::function<double(void)> m_aSpdFunc;

    bool m_fieldOrient;

    frc::SlewRateLimiter<units::scalar> m_xLimiter{3 / 1_s};
    frc::SlewRateLimiter<units::scalar> m_yLimiter{3 / 1_s};
    frc::SlewRateLimiter<units::scalar> m_aLimiter{3 / 1_s};
};