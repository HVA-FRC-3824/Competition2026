#include "lib/VisionPose.h"

VisionPose::VisionPose(std::string_view cameraName,
            frc::Transform3d            robotToCamPose,
            frc::AprilTagFieldLayout    tagLayout,
            Eigen::Matrix<double, 3, 1> singleTagStdDevs,
            Eigen::Matrix<double, 3, 1> multiTagStdDevs,
            frc::SwerveDrivePoseEstimator<4> *poseEstimator) :
    m_photonEstimator
    { 
        tagLayout,
        robotToCamPose
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
        m_visionSim->AddCamera(m_cameraSim.get(), robotToCamPose);
        m_cameraSim->EnableDrawWireframe(true);
    }
}

void VisionPose::Periodic()
{
    // Run each new pipeline result through our pose estimator
    for (const auto& result : m_camera.GetAllUnreadResults())
    {
        // cache result and update internal pose estimator
        m_latestResult = result;

        auto visionEst = m_photonEstimator.EstimateCoprocMultiTagPose(result);
        if (!visionEst)
        {
            visionEst = m_photonEstimator.EstimateLowestAmbiguityPose(result);
        }

        frc::SmartDashboard::PutBoolean("Vision is being called ", true);

        // In sim only, add our vision estimate to the sim debug field
        if (frc::RobotBase::IsSimulation()) 
        {
            if (visionEst)
            {
                GetSimDebugField()
                .GetObject("VisionEstimation")
                ->SetPose(visionEst->estimatedPose.ToPose2d());
            } 
            else 
            {
                GetSimDebugField().GetObject("VisionEstimation")->SetPoses({});
            }
        }

        if (visionEst)
        {
            static bool hasSeenAprilTag = true; // Should be false**
            if (hasSeenAprilTag)
            {
                auto stdDevs = GetEstimationStdDevs(visionEst->estimatedPose.ToPose2d());
                m_poseEstimator->AddVisionMeasurement(visionEst->estimatedPose.ToPose2d(), visionEst->timestamp, {stdDevs[0], stdDevs[1], stdDevs[2]});
            }
            else
            {
                frc::SmartDashboard::PutNumber("Pose Ambiguity ", visionEst.value().targetsUsed[0].poseAmbiguity);
                if (result.targets[0].poseAmbiguity < 0.2)
                {
                    hasSeenAprilTag = true;
                    m_poseEstimator->ResetPose(visionEst->estimatedPose.ToPose2d());
                }
            }
        }
    }
}

Eigen::Matrix<double, 3, 1> VisionPose::GetEstimationStdDevs(frc::Pose2d estimatedPose)
{
    Eigen::Matrix<double, 3, 1> estStdDevs = m_singleTagStdDevs;
    auto                        targets    = m_latestResult.GetTargets();
    int                         numTags    = 0;
    units::meter_t              avgDist    = 0_m;
    for (const auto &tgt : targets)
    {
        auto tagPose = m_photonEstimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
        if (tagPose)
        {
            numTags++;
            avgDist += tagPose->ToPose2d().Translation().Distance(estimatedPose.Translation());
        }
    }
    if (numTags == 0)
    {
        return estStdDevs;
    }
    avgDist /= numTags;
    if (numTags > 1)
    {
        estStdDevs = m_multiTagStdDevs;
    }
    if (numTags == 1 && avgDist > 4_m)
    {
        estStdDevs = (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max())
                        .finished();
    }
    else
    {
        estStdDevs = estStdDevs * (1 + (avgDist.value() * avgDist.value() / 30));
    }
    return estStdDevs;
}

void VisionPose::SimPeriodic(frc::Pose2d robotSimPose)
{
    m_visionSim->Update(robotSimPose);
}

void VisionPose::ResetSimPose(frc::Pose2d pose)
{
    if (frc::RobotBase::IsSimulation())
    {
        m_visionSim->ResetRobotPose(pose);
    }
}

frc::Field2d& VisionPose::GetSimDebugField() 
{ 
    return m_visionSim->GetDebugField(); 
}