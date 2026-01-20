#pragma once

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/estimation/VisionEstimation.h>
#include <photon/simulation/VisionSystemSim.h>
#include <photon/simulation/VisionTargetSim.h>
#include <photon/targeting/PhotonPipelineResult.h>

#include <functional>
#include <limits>
#include <memory>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

#include "Constants.h"


class VisionPose
{
    public:

        VisionPose(std::string_view           cameraName,
                    frc::Transform3d            robotToCamPose,
                    frc::AprilTagFieldLayout    tagLayout,
                    Eigen::Matrix<double, 3, 1> singleTagStdDevs,
                    Eigen::Matrix<double, 3, 1> multiTagStdDevs,
                    std::function<void(frc::Pose2d, units::second_t, Eigen::Matrix<double, 3, 1>)> estConsumer);
        
        photon::PhotonPipelineResult GetLatestResult() ;

        void Periodic();

        Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(frc::Pose2d estimatedPose);

        void SimPeriodic(frc::Pose2d robotSimPose);

        void ResetSimPose(frc::Pose2d pose);

        frc::Field2d& GetSimDebugField();

    private:

        std::string_view         cameraName;
        frc::Transform3d         robotToCamPose;
        frc::AprilTagFieldLayout tagLayout;

        photon::PhotonPoseEstimator photonEstimator;

        photon::PhotonCamera camera;

        Eigen::Matrix<double, 3, 1> singleTagStdDevs;
        Eigen::Matrix<double, 3, 1> multiTagStdDevs;

        std::unique_ptr<photon::VisionSystemSim>     visionSim;
        std::unique_ptr<photon::SimCameraProperties> cameraProp;
        std::shared_ptr<photon::PhotonCameraSim>     cameraSim;

        // The most recent result, cached for calculating std devs
        photon::PhotonPipelineResult                 m_latestResult;
        
        std::function<void(frc::Pose2d, units::second_t, Eigen::Matrix<double, 3, 1>)> estConsumer;
};
