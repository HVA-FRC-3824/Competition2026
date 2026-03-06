#include "lib/VisionPose.h"

#pragma Region VisionPose (Constructor)
/// @brief Constructor for VisionPose subsystem
/// @param cameraName Name of the camera as configured in PhotonVision
/// @param robotToCameraPose Transform from robot center to camera
/// @param tagLayout AprilTagFieldLayout representing the field's AprilTags
/// @param singleTagStdDevs Standard deviations for single tag pose estimation
/// @param multiTagStdDevs Standard deviations for multi-tag pose estimation
/// @param poseEstimator Pointer to the swerve drive pose estimator
VisionPose::VisionPose(std::string_view                  cameraName,
                       frc::Transform3d                  robotToCameraPose,
                       frc::AprilTagFieldLayout          tagLayout,
                       Eigen::Matrix<double, 3, 1>       singleTagStdDevs,
                       Eigen::Matrix<double, 3, 1>       multiTagStdDevs,
                       frc::SwerveDrivePoseEstimator<4> *poseEstimator) :
    m_photonEstimator
    { 
        tagLayout,
        robotToCameraPose
    },
    m_camera{cameraName},
    m_singleTagStdDevs{singleTagStdDevs},
    m_multiTagStdDevs{multiTagStdDevs},
    m_poseEstimator{poseEstimator}
{
    // Simulation setup, not really in a working state yet
    if (frc::RobotBase::IsSimulation())
    {
        m_visionSim = std::make_unique<photon::VisionSystemSim>("main");
        m_visionSim->AddAprilTags(tagLayout);
        m_cameraProp = std::make_unique<photon::SimCameraProperties>();
        m_cameraProp->SetCalibration(960, 720, frc::Rotation2d{90_deg});
        m_cameraProp->SetCalibError(.35, .10);
        m_cameraProp->SetFPS(15_Hz);
        m_cameraProp->SetAvgLatency(50_ms);
        m_cameraProp->SetLatencyStdDev(15_ms);
        m_cameraSim = std::make_shared<photon::PhotonCameraSim>(&m_camera, *m_cameraProp.get());
        m_visionSim->AddCamera(m_cameraSim.get(), robotToCameraPose);
        m_cameraSim->EnableDrawWireframe(true);
    }
}
#pragma endregion

#pragma Region Periodic
/// @brief Periodic method to be called from RobotPeriodic to process vision data
void VisionPose::Periodic()
{
    // Run each new pipeline result through our pose estimator
    for (const auto& result : m_camera.GetAllUnreadResults())
    {
        // cache result and update internal pose estimator
        m_latestResult = result;

        // Try to get a multi-tag pose estimate first
        auto visionEst = m_photonEstimator.EstimateCoprocMultiTagPose(result);

        // If that fails, try to get a single-tag pose estimate
        if (!visionEst)
        {
            // No multi-tag result, try single-tag
            visionEst = m_photonEstimator.EstimateLowestAmbiguityPose(result);
        }

        Log("Vision is being called ", true);

        // In sim only, add our vision estimate to the sim debug field
        if (frc::RobotBase::IsSimulation()) 
        {
            if (visionEst)
            {
                Log("VisionEstimation", visionEst->estimatedPose.ToPose2d());
            } 
            else 
            {
                Log("VisionEstimation", frc::Pose2d{});
            }
        }

        // If a valid pose estimate was found, add it to the pose estimator
        if (visionEst)
        {
            static bool hasSeenAprilTag = false;

            // If an AprilTag has been seen before, add vision measurement normally
            if (hasSeenAprilTag)
            {
                // Add vision measurement to pose estimator with calculated standard deviations
                auto stdDevs = GetEstimationStdDevs(visionEst->estimatedPose.ToPose2d());
                m_poseEstimator->AddVisionMeasurement(visionEst->estimatedPose.ToPose2d(), visionEst->timestamp, {stdDevs[0], stdDevs[1], stdDevs[2]});
            }
            else
            {
                Log("Pose Ambiguity ", visionEst.value().targetsUsed[0].poseAmbiguity);

                // First time seeing an AprilTag, only reset pose if ambiguity is low
                if (result.targets[0].poseAmbiguity < 0.2)
                {
                    hasSeenAprilTag = true;
                    m_poseEstimator->ResetPose(visionEst->estimatedPose.ToPose2d());
                }
            }
        }
    }
}
#pragma endregion

#pragma Region GetEstimationStdDevs
/// @brief Calculate standard deviations for vision pose estimate based on number of tags and distance
/// @param estimatedPose The estimated pose from vision
/// @return Eigen matrix of standard deviations [stddev_x, stddev_y, stddev_theta]
Eigen::Matrix<double, 3, 1> VisionPose::GetEstimationStdDevs(frc::Pose2d estimatedPose)
{
    Eigen::Matrix<double, 3, 1> estStdDevs = m_singleTagStdDevs;
    auto                        targets    = m_latestResult.GetTargets();
    int                         numTags    = 0;
    units::meter_t              avgDist    = 0_m;

    // Calculate number of tags used and average distance to tags
    for (const auto &tgt : targets)
    {
        // Get the pose of the tag from the field layout
        auto tagPose = m_photonEstimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());

        // If the tag pose is valid, update counters
        if (tagPose)
        {
            // Update counters for valid tag poses
            numTags++;

            // Accumulate distance to tag
            avgDist += tagPose->ToPose2d().Translation().Distance(estimatedPose.Translation());
        }
    }

    // If no tags were seen, return max stddevs
    if (numTags == 0)
        return estStdDevs;

    // Calculate average distance
    avgDist /= numTags;

    // Adjust standard deviations based on number of tags and distance
    if (numTags > 1)
    {
        // Use multi-tag standard deviations
        estStdDevs = m_multiTagStdDevs;
    }

    // Adjust standard deviations for single tag at long distance
    if (numTags == 1 && avgDist > 4_m)
    {
        // If only one tag and distance is greater than 4 meters, set to max stddevs
        estStdDevs = (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max())
                        .finished();
    }
    else
    {
        // Scale standard deviations based on distance squared over 30
        estStdDevs = estStdDevs * (1 + (avgDist.value() * avgDist.value() / 30));
    }

    // Return calculated standard deviations
    return estStdDevs;
}
#pragma endregion

#pragma Region SimPeriodic
/// @brief Simulation periodic method to update vision sim with robot pose
/// @param robotSimPose The simulated robot pose on the field
void VisionPose::SimPeriodic(frc::Pose2d robotSimPose)
{
    // Update the vision system simulation with the robot's simulated pose
    m_visionSim->Update(robotSimPose);
}
#pragma endregion

#pragma Region ResetSimPose
/// @brief Reset the simulated robot pose in the vision simulation
/// @param pose The pose to reset the simulated robot to
void VisionPose::ResetSimPose(frc::Pose2d pose)
{
    // Reset the robot pose in simulation
    if (frc::RobotBase::IsSimulation())
    {
        m_visionSim->ResetRobotPose(pose);
    }
}
#pragma endregion

#pragma Region GetSimDebugField
/// @brief Get the simulated debug field for visualization
/// @return Reference to the simulated debug field
frc::Field2d& VisionPose::GetSimDebugField() 
{ 
    // Return the debug field from the vision simulation
    return m_visionSim->GetDebugField(); 
}
#pragma endregion
