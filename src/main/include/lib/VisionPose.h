#pragma once

#pragma region Includes
#include <functional>
#include <limits>
#include <memory>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/simulation/VisionSystemSim.h>
#include <photon/targeting/PhotonPipelineResult.h>

#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/RobotBase.h>

#include "lib/Logging.h"

#include "Constants.h"
#pragma endregion

class VisionPose
{
    public:

        VisionPose(std::string_view                  cameraName,
                   frc::Transform3d                  robotToCameraPose,
                   frc::AprilTagFieldLayout          tagLayout,
                   Eigen::Matrix<double, 3, 1>       singleTagStdDevs,
                   Eigen::Matrix<double, 3, 1>       multiTagStdDevs,
                   frc::SwerveDrivePoseEstimator<4> *poseEstimator);
        
        void                         Periodic();

        Eigen::Matrix<double, 3, 1>  GetEstimationStdDevs(frc::Pose2d estimatedPose);

        void                         SimPeriodic(frc::Pose2d robotSimPose);

        void                         ResetSimPose(frc::Pose2d pose);

        frc::Field2d                &GetSimDebugField();

    private:

        photon::PhotonPoseEstimator                  m_photonEstimator;

        photon::PhotonCamera                         m_camera;

        Eigen::Matrix<double, 3, 1>                  m_singleTagStdDevs;
        Eigen::Matrix<double, 3, 1>                  m_multiTagStdDevs;

        std::unique_ptr<photon::VisionSystemSim>     m_visionSim;
        std::unique_ptr<photon::SimCameraProperties> m_cameraProp;
        std::shared_ptr<photon::PhotonCameraSim>     m_cameraSim;

        photon::PhotonPipelineResult                 m_latestResult;
        
        frc::SwerveDrivePoseEstimator<4U>           *m_poseEstimator;
};
