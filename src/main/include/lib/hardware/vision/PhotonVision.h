// #pragma once

// #include <limits>
// #include <memory>

// #include <photon/PhotonCamera.h>
// #include <photon/PhotonPoseEstimator.h>

// #include <photon/estimation/VisionEstimation.h>

// #include <photon/simulation/VisionSystemSim.h>
// #include <photon/simulation/VisionTargetSim.h>

// #include <photon/targeting/PhotonPipelineResult.h>

// #include <frc/RobotBase.h>
// #include <frc2/command/SubsystemBase.h>

// #include "VisionData.h"


// class PhotonVision : public frc2::SubsystemBase
// {
//     public:
    
//         PhotonVision(PhotonVision const&)    = delete;
//         void operator=(PhotonVision const&)  = delete;

//         static PhotonVision* GetInstance();

//         void Periodic() override;

//         std::optional<std::pair<frc::Pose2d, units::second_t>> GetResult();

//         wpi::array<double, 3> GetEstimationStdDevs(frc::Pose2d estimatedPose);

//         frc::Field2d& GetSimDebugField() { return visionSim->GetDebugField(); }

//         void SimPeriodic(frc::Pose2d robotSimPose);

//         void ResetSimPose(frc::Pose2d pose);

//         frc::Pose2d GetNearestTag();

//     private:

//         PhotonVision();

//         photon::PhotonPoseEstimator photonEstimator{
//             constants::vision::TagLayout,
//             photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
//             constants::vision::RobotToCam};

//         photon::PhotonCamera camera{constants::vision::CameraName};

//         std::unique_ptr<photon::VisionSystemSim> visionSim;

//         std::unique_ptr<photon::SimCameraProperties> cameraProp;

//         std::shared_ptr<photon::PhotonCameraSim> cameraSim;

//         std::optional<std::pair<frc::Pose2d, units::second_t>> m_lastEstimate;

//         // The most recent result, cached for calculating std devs
//         photon::PhotonPipelineResult m_latestResult;
// };