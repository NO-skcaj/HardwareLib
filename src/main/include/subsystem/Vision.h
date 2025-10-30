#pragma once

#include "lib/hardware/vision/PhotonVision.h"


// class Vision : public hardware::vision::PhotonVision, public frc2::SubsystemBase
// {
//     public:

//         static Vision* GetInstance()
//         {
//             static Vision instance;
//             static Vision* instancePtr;
//             return instancePtr;
//         }

//         inline void Periodic() override
//         {
//             // This function has some side effects that work to what we want
//             std::pair<frc::Pose2d, units::time::second_t> visionEst = GetEstimate();
//         }

//         inline std::pair<frc::Pose2d, units::time::second_t> GetResult()
//         {
//             std::pair<frc::Pose2d, units::time::second_t> visionEst = GetEstimate();

//             return visionEst;
//         }

//         frc::Pose2d GetNearestTag()
//         {
//             return GetResult().first.Nearest(constants::vision::AprilTagLocations::Pose2dTagsSpan);
//         }

//     private:

//         Vision()
//             : PhotonVision()
//         {}

// };