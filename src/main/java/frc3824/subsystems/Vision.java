package frc3824.subsystems;
// Graciously professionally stolen from 3966

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3824.Constants.VisionConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;


public class Vision extends SubsystemBase {
    private static final PhotonCamera m_camera1 = new PhotonCamera(VisionConstants.kCameraName1);
    private static final PhotonCamera m_camera2 = new PhotonCamera(VisionConstants.kCameraName2);

    private static PhotonPipelineResult m_result1 = null;
    private static PhotonPipelineResult m_result2 = null;

    private static final PhotonPoseEstimator m_poseEstimator1 = new PhotonPoseEstimator(
        VisionConstants.kTagLayout, VisionConstants.RobotToCam1);
    private static final PhotonPoseEstimator m_poseEstimator2 = new PhotonPoseEstimator(
        VisionConstants.kTagLayout, VisionConstants.kRobotToCam2);

    @Override
    public void periodic() {
        var results1 = m_camera1.getAllUnreadResults();
        if (!results1.isEmpty()){
            m_result1 = results1.get(results1.size() - 1);
        }
        var results2 = m_camera2.getAllUnreadResults();
        if (!results2.isEmpty()){
            m_result2 = results2.get(results2.size() - 1);
        }
    }

    public static PhotonPipelineResult getm_result1() {
        return m_result1;
    }

    public static PhotonPipelineResult getm_result2() {
        return m_result2;
    }

    public static PhotonCamera getm_camera1() {
        return m_camera1;
    }

    public static PhotonCamera getm_camera2() {
        return m_camera2;
    }
    
    public static boolean resultHasTargets() {
        return (m_result1 != null && m_result1.hasTargets()) || (m_result2 != null && m_result2.hasTargets());
    }

    public static int[] tagsInFrame() {
        List<PhotonTrackedTarget> targets = getAllTargets();
        int[] tags = new int[targets.size()];
        for (int i = 0; i < targets.size(); i++) {
            tags[i] = targets.get(i).getFiducialId();
        }
        return tags;
    }

    public static List<PhotonTrackedTarget> getAllTargets() {
        List<PhotonTrackedTarget> allTargets = new ArrayList<>();
        if (m_result1 != null && m_result1.hasTargets()) {
            allTargets.addAll(m_result1.getTargets());
        }
        if (m_result2 != null && m_result2.hasTargets()) {
            allTargets.addAll(m_result2.getTargets());
        }
        return allTargets;
    }

    public static int getBestTag() {
        if (m_result1 != null && m_result1.hasTargets()) {
            return m_result1.getBestTarget().getFiducialId();
        }
        if (m_result2 != null && m_result2.hasTargets()) {
            return m_result2.getBestTarget().getFiducialId();
        }
        return 0;
    }

    public static Optional<EstimatedRobotPose> getEstimatedGlobalPoseCam1(PhotonPipelineResult result, Pose2d referencePose) {
        if (result == null || !result.hasTargets()){
            return Optional.empty();
        }

        Pose3d[] usedTags = new Pose3d[result.targets.size()];
        for (int i = 0; i < result.targets.size(); i++){
          usedTags[i] = (VisionConstants.kTagLayout.getTagPose(result.targets.get(i).fiducialId).get());
        }

        Logger.recordOutput("PoseEst/Camera 1 Tags Used", usedTags);

        Optional<EstimatedRobotPose> update = m_poseEstimator1.estimateClosestToReferencePose(result, new Pose3d(referencePose));
            
        
        return update;
    }

    public static Matrix<N3, N1> updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {

        Matrix<N3, N1> curStdDevs = VisionConstants.kSingleTagStdDevs;
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.kSingleTagStdDevs;
        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = VisionConstants.kTagLayout.getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible.
                curStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 2)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                // TODO Tweak the constant here PLEASE to change how much we trust multi tag as distances increase
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
        return curStdDevs;
    }



    public static Optional<EstimatedRobotPose> getEstimatedGlobalPoseCam2(Pose2d prevEstimatedRobotPose, PhotonPipelineResult result) {
        if (result == null || !result.hasTargets()){
            return Optional.empty();
        }

        Optional<EstimatedRobotPose> update = Optional.empty();

        Pose3d[] usedTags = new Pose3d[result.targets.size()];
        for (int i = 0; i < result.targets.size(); i++){
          usedTags[i] = (VisionConstants.kTagLayout.getTagPose(result.targets.get(i).fiducialId).get());
        }

        Logger.recordOutput("PoseEst/Camera 2 Tags Used", usedTags);

        
        update = m_poseEstimator2.estimateClosestToCameraHeightPose(result);
        
        return update;
    }

    public static double targetYaw(int targetNumber) {
        for (PhotonTrackedTarget target : getAllTargets()) {
            if (target.getFiducialId() == targetNumber) {
                return target.getYaw();
            }
        }
        return 0;
    }

    public static Transform3d targetTransform(int targetNumber) {
        for (PhotonTrackedTarget target : getAllTargets()) {
            if (target.getFiducialId() == targetNumber) {
                return target.getBestCameraToTarget();
            }
        }
        return new Transform3d();
    }

    public static PhotonTrackedTarget returnTag(int targetNumber) {
        for (PhotonTrackedTarget target : getAllTargets()) {
            if (target.getFiducialId() == targetNumber) {
                return target;
            }
        }
        return new PhotonTrackedTarget(); // Empty target
    }
}