#include "Constants.h"

namespace Swerve
{
    namespace System
    {
        frc::SwerveDriveKinematics<4> kDriveKinematics {
            frc::Translation2d{+kWheelBase/2,+kTrackWidth/2},
            frc::Translation2d{+kWheelBase/2,-kTrackWidth/2},
            frc::Translation2d{-kWheelBase/2,+kTrackWidth/2},
            frc::Translation2d{-kWheelBase/2,-kTrackWidth/2} };
    } // namespace System
    namespace Auto
    {
        frc::TrapezoidProfile<units::radians>::Constraints kThetaControllerConstraints{
            kMaxAngularSpeed,
            kMaxAngularAcceleration};
    }
} // namespace Swerve