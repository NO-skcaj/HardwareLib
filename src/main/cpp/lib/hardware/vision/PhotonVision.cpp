// #include "lib/hardware/vision/PhotonVision.h"


// PhotonVision* PhotonVision::GetInstance() 
// {
//     static PhotonVision instance;
//     static PhotonVision* instancePtr = &instance;
//     return instancePtr;
// }

// PhotonVision::PhotonVision() 
// {
//     photonEstimator.SetMultiTagFallbackStrategy(photon::PoseStrategy::LOWEST_AMBIGUITY);

//     if (frc::RobotBase::IsSimulation()) 
//     {
//         visionSim = std::make_unique<photon::VisionSystemSim>("main");

//         visionSim->AddAprilTags(constants::vision::TagLayout);

//         cameraProp = std::make_unique<photon::SimCameraProperties>();

//         cameraProp->SetCalibration(960, 720, frc::Rotation2d{90_deg});
//         cameraProp->SetCalibError(.35, .10);
//         cameraProp->SetFPS(15_Hz);
//         cameraProp->SetAvgLatency(50_ms);
//         cameraProp->SetLatencyStdDev(15_ms);

//         cameraSim = std::make_shared<photon::PhotonCameraSim>(&camera, *cameraProp.get());

//         visionSim->AddCamera(cameraSim.get(), constants::vision::RobotToCam);
//         cameraSim->EnableDrawWireframe(true);

//         frc::SmartDashboard::PutData("VisionSimField", &visionSim->GetDebugField());
//     }
// }

// void PhotonVision::Periodic()
// {
//     std::optional<photon::EstimatedRobotPose> visionEst;

//     // Run each new photon::PhotonPipelineResult result through our pose estimator
//     auto results = camera.GetAllUnreadResults();
//     for (const auto& result : results)
//     {
//         m_latestResult = result;

//         // cache result and update pose estimator
//         visionEst = photonEstimator.Update(result);

//         // In sim only, add our vision estimate to the sim debug field
//         if (frc::RobotBase::IsSimulation()) 
//         {
//             if (visionEst) 
//             {
//                 GetSimDebugField()
//                     .GetObject("VisionEstimation")
//                     ->SetPose(visionEst->estimatedPose.ToPose2d());
//             } else {
//                 GetSimDebugField().GetObject("VisionEstimation")->SetPoses({});
//             }
//         }
//     }

//     // Add the vision measurement to the odometry
//     // Check if the vision estimate is valid
//     if (visionEst)
//         m_lastEstimate = std::optional<std::pair<frc::Pose2d, units::second_t>>{{visionEst.value().estimatedPose.ToPose2d(), visionEst.value().timestamp}};

//     m_lastEstimate = std::nullopt;
// }

// std::optional<std::pair<frc::Pose2d, units::second_t>> PhotonVision::GetResult()
// {
//     return m_lastEstimate;
// }

// wpi::array<double, 3> PhotonVision::GetEstimationStdDevs(frc::Pose2d estimatedPose) 
// {
//     Eigen::Matrix<double, 3, 1> estStdDevs =
//         constants::vision::SingleTagStdDevs;
//     auto targets = m_latestResult.GetTargets();
//     int numTags = 0;
//     units::meter_t avgDist = 0_m;
//     for (const auto& tgt : targets) 
//     {
//         auto tagPose =
//             photonEstimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
//         if (tagPose) 
//         {
//         numTags++;
//         avgDist += tagPose->ToPose2d().Translation().Distance(
//             estimatedPose.Translation());
//         }
//     }
//     if (numTags == 0) 
//     {
//         return {estStdDevs[0], estStdDevs[1], estStdDevs[2]};
//     }
//     avgDist /= numTags;
//     if (numTags > 1) 
//     {
//         estStdDevs = constants::vision::MultiTagStdDevs;
//     }
//     if (numTags == 1 && avgDist > 4_m) 
//     {
//         estStdDevs = (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
//                       std::numeric_limits<double>::max(),
//                       std::numeric_limits<double>::max()).finished();
//     } else {
//         estStdDevs = estStdDevs * (1 + (avgDist.value() * avgDist.value() / 30));
//     }
//     return {estStdDevs[0], estStdDevs[1], estStdDevs[2]};
// }

// void PhotonVision::SimPeriodic(frc::Pose2d robotSimPose) 
// {
//     this->visionSim->Update(robotSimPose);
// }

// void PhotonVision::ResetSimPose(frc::Pose2d pose) 
// {
//     if (frc::RobotBase::IsSimulation()) 
//     {
//         this->visionSim->ResetRobotPose(pose);
//     }
// }

// frc::Pose2d PhotonVision::GetNearestTag()
// {
//     return GetResult().value_or(std::pair<frc::Pose2d, units::second_t>{frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}, 0_s}).first.Nearest(constants::vision::AprilTagLocations::Pose2dTagsSpan);
// }