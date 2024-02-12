#pragma once

#include "subsystems/SwerveSubsystem.h"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Commands.h>

#include <frc/filter/SlewRateLimiter.h>

#include <units/time.h>

class SwerveTesterCommand : public frc2::CommandHelper<frc2::Command,SwerveTesterCommand>
{
public:
    /**
     * Constructs a command that does a speed ramp in the range 0 and kPhysicalMoveMax and finishes itself when done
     * 
     * @param subsystem* Point const to a swerve subsystem to capture 
     * the requirements so other commands don't run on same subsystem
     * @param bDirRamp bool determines direction of the ramp: true 
    */
    SwerveTesterCommand(SwerveSubsystem* const subsytem,bool bDirRamp,units::second_t time);
    //* Command overloaded function: Executed once when first being scheduled
    void Initialize() override;
    //* Command overloaded function: Executes every command schedule pass if has access to subsystem
    void Execute() override;
    /** 
     * Command overloaded function: Executes after execute()
     * 
     * @return bool that unschedules the command if true
    */
    bool IsFinished() override;
    /** 
     * Command overloaded function: Executes every command schedule pass if another command obtains subsystem
     * 
     * @param interrupt bool that is true if another command obtained subsystem requirement or explictly cancelled
    */
    void End(bool bInterupted) override;
private:
    //* Pointer const to swerve subsystem
    SwerveSubsystem* const m_subsystem;
    //* Current speed
    double m_speed;
    //* Direction of speed ramp
    bool m_direction;
    //* Limits the rate of change
    frc::SlewRateLimiter<units::scalar> m_speeder;
};