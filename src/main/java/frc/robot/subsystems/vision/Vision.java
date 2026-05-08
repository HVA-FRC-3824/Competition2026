package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.Robot;

import frc.robot.lib.VisionData;
import frc.robot.lib.Module.Logged;
import frc.robot.lib.Module;
import frc.robot.lib.Util;

public class Vision extends Module<Vision.Outputs> {

  public static AprilTagFieldLayout fieldAprilTags = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  private VisionSystemSim visionSim;

  private SwerveDriveSimulation m_swerveSim;

  ArrayList<VisionData> m_data = new ArrayList<>();

  public Camera[] cameras = new Camera[] {
    new Camera(Constants.Vision.kCameraName1, Constants.Vision.RobotToCam1)
  };

  public Vision(SwerveDriveSimulation swerveSim) {

    m_outputs = Outputs.zeroed();

    m_swerveSim = swerveSim;
    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(fieldAprilTags);

      for (Camera m_camera : cameras) {
      visionSim.addCamera(m_camera.getSimCamera(), m_camera.getOffset());
      }
    }
  }

  public void clearData() {

    m_data.clear();
  }

  @Override
  public void updateOutputs() {

    for (Camera m_camera : cameras) m_data.addAll(m_camera.getMeasurements());

    if (Robot.isSimulation()) visionSim.update(m_swerveSim.getSimulatedDriveTrainPose());
    
    m_outputs = new Outputs(
      m_data,
      (m_data.size() > 0) ? Util.getTagPose(m_data.get(Math.max(0, m_data.size() - 1)).target()) : m_outputs.lastSeenTag()
    );
  }

  public static class Camera {

    private Matrix<N3, N1> curStdDevs = Constants.Vision.kSingleTagStdDevs;

    private int m_xRes = 1080;
    private int m_yRes = 720;
    private double m_fov = 70;
    private double m_fps = 15;

    private PhotonCamera        m_camera;
    private PhotonCameraSim     m_simCamera;
    private PhotonPoseEstimator estimator;

    private Transform3d m_offset;

    boolean m_ignored;

    public Camera(String name, Transform3d m_offset) {

      m_camera = new PhotonCamera(name);
      estimator = new PhotonPoseEstimator(fieldAprilTags, m_offset);

      this.m_offset = m_offset;

      if (Robot.isSimulation()) {

        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(m_xRes, m_yRes, Rotation2d.fromDegrees(m_fov));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(m_fps);
        cameraProp.setAvgLatencyMs(50);
        cameraProp.setLatencyStdDevMs(15);

        m_simCamera = new PhotonCameraSim(m_camera, cameraProp);

        m_simCamera.enableDrawWireframe(true);
      }
    }

    public ArrayList<VisionData> getMeasurements() {

      ArrayList<VisionData> measurements = new ArrayList<>();
      for (var result : m_camera.getAllUnreadResults()) {

        Optional<EstimatedRobotPose> visionEst = estimator.estimateCoprocMultiTagPose(result);
        if (visionEst.isEmpty())     visionEst = estimator.estimateLowestAmbiguityPose(result);

        updateEstimationStdDevs(visionEst, result.getTargets());

        if (visionEst.isEmpty()) continue;

        EstimatedRobotPose est = visionEst.get();
        if (est.targetsUsed.size() == 0) continue;
        measurements.add(new VisionData(est.estimatedPose.toPose2d(), est.timestampSeconds, curStdDevs, est.targetsUsed.get(0).fiducialId));
      }
      return measurements;
    }

    public PhotonCameraSim getSimCamera() {
      return m_simCamera;
    }

    public Transform3d getOffset() {
      return m_offset;
    }

    /// PHOTONVISION PROVIDED 
    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this m_camera frame
     */
    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
      if (estimatedPose.isEmpty()) {
        // No pose input. Default to single-tag std devs
        curStdDevs = Constants.Vision.kSingleTagStdDevs;
      } else {
        // Pose present. Start running Heuristic
        var estStdDevs = Constants.Vision.kSingleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an average-distance metric
        for (var tgt : targets) {
          var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
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
          // No tags visible. Default to single-tag std devs
          curStdDevs = Constants.Vision.kSingleTagStdDevs;
        } else {
          // One or more tags visible, run the full heuristic.
          avgDist /= numTags;
          // Decrease std devs if multiple targets are visible
          if (numTags > 1) estStdDevs = Constants.Vision.kMultiTagStdDevs;
          // Increase std devs based on (average) distance
          // max distance 15 meters
          if (numTags == 1 && avgDist > 15) {
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          } else { 
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          }
          curStdDevs = estStdDevs;
        }
      }
    }
  }

  public static record Outputs(
    ArrayList<VisionData> measurements,
    Pose2d lastSeenTag
  ) implements Logged {

    public static Outputs zeroed() {

      return new Outputs(new ArrayList<VisionData>(0), new Pose2d());
    }

    @Override
    public void log() {

      if (measurements.size() > 0) Logger.recordOutput("Vision/measurement", measurements.get(measurements.size()-1).visionMeasurement());
      Logger.recordOutput("Vision/Last Seen Tag", lastSeenTag);
    }
  }
}