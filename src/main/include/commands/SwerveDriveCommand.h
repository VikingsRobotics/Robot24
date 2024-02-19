#pragma once

#include "subsystems/SwerveSubsystem.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/filter/SlewRateLimiter.h>

#include <functional>

class SwerveDriveCommand : public frc2::CommandHelper<frc2::Command,SwerveDriveCommand> {
public:
    /**
     * Constructs a command that rate limits the input to swerve modules to prevent damage
     * 
     * @param subsystem* pointer const to a swerve subsystem to capture 
     * the requirements so other commands don't run on same subsystem
     * @param xSpdFunc function that returns [-1,1] for x (left,right) axis speed of 
     * swerve subsystem multiplied by kDriveMoveSpeedMax for speed
     * @param ySpdFunc function that returns [-1,1] for y (forward,backward) axis speed of 
     * swerve subsystem multiplied by kDriveMoveSpeedMax for speed
     * @param aSpdFunc function that returns [-1,1] for angluar speed of swerve subsystem
     * multiplied by kDriveAngleSpeedMax for speed
     * @param brakeFunc function that returns [true,false] (default: false) for if swerve subsystem
     * should stop all motion 
     * @param fieldFunc function that returns [true,false] (default: true) for if swerve subsystem
     * are field centric
     * @param rateLimit double (default: 3) that determines how quickly output equals input,
     * 1 = 100% per sec
    */
    SwerveDriveCommand(SwerveSubsystem* const subsystem,std::function<double(void)> xSpdFunc,
        std::function<double(void)> ySpdFunc,std::function<double(void)> aSpdFunc,
        std::function<bool(void)> brakeFunc = []{return false;},std::function<bool(void)> fieldFunc = []{return true;},
        double rateLimit = 3);
    //* Command overloaded function: Executes every command schedule pass if has access to subsystem
    void Execute() override;
    /** 
     * Command overloaded function: Executes every command schedule pass if another command obtains subsystem
     * 
     * @param interrupt bool that is true if another command obtained subsystem requirement or explictly cancelled
    */
    void End(bool interrupted) override;
    
private:
    //* Pointer const to swerve subsystem
    SwerveSubsystem* const m_subsystem;
    //* Retrieves input from user every time executed
    std::function<double(void)> m_xSpdFunc;
    //* Retrieves input from user every time executed
    std::function<double(void)> m_ySpdFunc;
    //* Retrieves input from user every time executed
    std::function<double(void)> m_aSpdFunc;
    //* Retrieves input from user every time executed
    std::function<bool(void)> m_brakeFunc;
    //* Retrieves input from user every time executed
    std::function<bool(void)> m_fieldFunc;
    //* Limits the amount of change over seconds
    frc::SlewRateLimiter<units::scalar> m_xLimiter;
    //* Limits the amount of change over seconds
    frc::SlewRateLimiter<units::scalar> m_yLimiter;
    //* Limits the amount of change over seconds
    frc::SlewRateLimiter<units::scalar> m_aLimiter;
};