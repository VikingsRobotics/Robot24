#pragma once

#include "subsystems/SwerveSubsystem.h"

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/SwerveControllerCommand.h>

#include <units/angle.h>

#include "Constants.h"

namespace Auto
{
    class AutoMoveCommand : public frc2::SwerveControllerCommand<4>
    {
    public:
        AutoMoveCommand(SwerveSubsystem* const subsystem,const frc::Trajectory traj);
    private:
        SwerveSubsystem* const m_subsystem;
        frc::PIDController m_xController{Swerve::Auto::kPXController,0,0};
        frc::PIDController m_yController{Swerve::Auto::kPYController,0,0};
        frc::ProfiledPIDController<units::radian_t> m_aController{Swerve::Auto::kPAController,0.0,0.0,
            frc::TrapezoidProfile<units::radian_t>::Constraints{Swerve::Auto::kMaxVelocity,Swerve::Auto::kMaxAcceleration}};
    };
} // namespace auto
