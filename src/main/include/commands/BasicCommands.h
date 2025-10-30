#pragma once

#include <functional>

#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>

#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/WaitCommand.h>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>

#include "subsystem/Swerve.h"
#include "subsystem/Volcano.h"
#include "subsystem/Gyro.h"

#include "Constants.h"

// DRIVEEEEEEEEEEEE

inline frc2::CommandPtr ChassisZeroHeading(Gyro* instance)
{
    return frc2::FunctionalCommand{
        [] () { },
        [] () {  },
        [instance] (bool interupted) { instance->ResetYaw(); },
        [] () { return true; },
        {}
    }.ToPtr();
}

inline frc2::CommandPtr ChassisDrive(Swerve* instance, std::function<frc::ChassisSpeeds()> chassisSpeedsSupplier)
{
    return frc2::FunctionalCommand{
        []                      ()                { },
        [instance, chassisSpeedsSupplier] ()                { instance->Drive(chassisSpeedsSupplier()); },
        []                      (bool interupted) { },
        []                      ()                { return false; },
        {instance}
    }.ToPtr();
}

inline frc2::CommandPtr FlipFieldCentricity(Swerve* instance)
{
    return frc2::FunctionalCommand{
        []         ()                { },
        [instance] ()                { instance->FlipFieldCentric(); },
        []         (bool interupted) { },
        []         ()                { return true; },
        {instance}
    }.ToPtr();
}

// Path following commands

inline frc2::CommandPtr ChassisDrivePose(Swerve* instance, std::string CommandName)
{
    // return AutoBuilder::followPath(PathPlannerPath::fromPathFile(CommandName));
    return frc2::WaitCommand(0.1_s).ToPtr(); // Temporary fix for pathplanner not working correctly when called immediately after another command

}

inline frc2::CommandPtr ChassisDrivePose(Swerve* instance, frc::Pose2d targetPose) // End goal state relative to the origin, blue alliance side
{
    // return AutoBuilder::pathfindToPose(targetPose, constants::PathPlanner::Constraints);
    return frc2::WaitCommand(0.1_s).ToPtr(); // Temporary fix for pathplanner not working correctly when called immediately after another command
}

// // This command will align the robot to the nearest AprilTag
// // It will use the AprilTag's pose to determine the target position and rotation
// // The robot will drive towards the target position and rotate to face the target rotation
// inline frc2::CommandPtr AlignToNearestTag(Swerve* instance, frc::Transform2d targetOffset)
// {
//     // This doesn't need to be a variable. When I wrote this, I just really liked using lambdas. Now, it kinda needs to be because its not in its own class
//     std::function<frc::Pose2d(frc::Pose2d, frc::Transform2d)> getTargetWithOffset = 
//         [] (frc::Pose2d targetPosition, frc::Transform2d targetOffset)
//         {
//             // Rotate offset
//             return frc::Pose2d{
//                 targetPosition.X() +                  targetOffset.Translation().X() * std::cos(targetPosition.Rotation().Radians().value()) - targetOffset.Translation().Y() * std::sin(targetPosition.Rotation().Radians().value()),
//                 targetPosition.Y() +                  targetOffset.Translation().X() * std::sin(targetPosition.Rotation().Radians().value()) + targetOffset.Translation().Y() * std::cos(targetPosition.Rotation().Radians().value()),
//                 targetPosition.Rotation().Degrees() + targetOffset.Rotation().Degrees()};
//         };

//     return ChassisDrivePose(getTargetWithOffset(PhotonVision::GetInstance()->GetNearestTag(), targetOffset));
// }

// VOLCANOOOOO

inline frc2::CommandPtr VolcanoFlywheelOn(Volcano* instance)
{
    return frc2::FunctionalCommand{
        []         ()                { },
        [instance] ()                { instance->SetFlywheel(true); },
        []         (bool interupted) { },
        []         ()                { return true; },
        {instance}
    }.ToPtr();
}

inline frc2::CommandPtr VolcanoFlywheelOff(Volcano* instance)
{
    return frc2::FunctionalCommand{
        []         ()                { },
        [instance] ()                { instance->SetFlywheel(false); },
        []         (bool interupted) { },
        []         ()                { return true; },
        {instance}
    }.ToPtr();
}

inline frc2::CommandPtr VolcanoIndexerControl(Volcano* instance, std::function<double()> arbitaryGetter)
{
    return frc2::FunctionalCommand{
        []                           ()                { },
        [instance, arbitaryGetter]   ()                { instance->SetIndexer(arbitaryGetter()); },
        []                           (bool interupted) { },
        []                           ()                { return false; },
        {instance}
    }.ToPtr();
}