package frc.robot.lib.pathing;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import java.util.ArrayList;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.lib.Util;

public class CustomPathing {
    
    private final ProfiledPIDController m_translationController;
    private final ProfiledPIDController m_rotationController;

    private final Distance m_lookAheadDistance;
    private final Distance m_interpolationDistance;

    public CustomPathing(
        ProfiledPIDController xyController, ProfiledPIDController rotController,
        Distance translationTolerance, Angle rotationTolerance, 
        Distance interpolationDistance, Distance lookAheadDistance
    ) {
        m_translationController = xyController;
        m_rotationController    = rotController;
        m_translationController.setTolerance(translationTolerance.in(Meters));
        m_rotationController.setTolerance(rotationTolerance.in(Radians));

        m_interpolationDistance = interpolationDistance;
        m_lookAheadDistance = lookAheadDistance;
    }

    public Controller fromWaypoints(Pose2d... poses) {
        return new Controller(m_interpolationDistance, m_lookAheadDistance, m_translationController, m_rotationController, poses);
    }

    public static class Controller {
        
        private final ArrayList<Pose2d> m_path = new ArrayList<>();

        private final ProfiledPIDController m_xController;
        private final ProfiledPIDController m_yController;
        private final ProfiledPIDController m_rotationController;
        
        private final Distance m_interpolationDistance;
        private final Distance m_lookAheadDistance;

        public Controller(Distance interpolationDistance, Distance lookAheadDistance, ProfiledPIDController xyController, ProfiledPIDController rotController, Pose2d... poses) {
            m_lookAheadDistance     = lookAheadDistance;
            m_interpolationDistance = interpolationDistance;

            // Create copies of the XY controller
            m_xController = new ProfiledPIDController(xyController.getP(), xyController.getI(), xyController.getD(), xyController.getConstraints());
            m_yController = new ProfiledPIDController(xyController.getP(), xyController.getI(), xyController.getD(), xyController.getConstraints());
            m_xController.setTolerance(xyController.getPositionTolerance());
            m_yController.setTolerance(xyController.getPositionTolerance());
            m_rotationController = rotController;

            for (int poseIdx = 0; poseIdx < poses.length - 1; poseIdx++) {
                m_path.add(poses[poseIdx]);
                Distance dist = Util.dist(poses[poseIdx], poses[poseIdx+1]);
                
                double interpolationPoints = dist.in(Meters) / m_interpolationDistance.in(Meters);
                for (double lerpIdx = 0; lerpIdx < interpolationPoints; lerpIdx++) {
                    Distance midX   = Meters.of(MathUtil.interpolate(poses[poseIdx].getMeasureX().in(Meters), poses[poseIdx+1].getMeasureX().in(Meters), lerpIdx / interpolationPoints));
                    Distance midY   = Meters.of(MathUtil.interpolate(poses[poseIdx].getMeasureY().in(Meters), poses[poseIdx+1].getMeasureY().in(Meters), lerpIdx / interpolationPoints));
                    Angle    midRot = poses[poseIdx].getRotation().interpolate(poses[poseIdx+1].getRotation(), lerpIdx / interpolationPoints).getMeasure();
                    
                    m_path.add(new Pose2d(midX, midY, new Rotation2d(midRot)));
                } // For each mid point
            } // For each pose
            m_path.add(poses[poses.length - 1]);
        }

        public ChassisSpeeds getSpeeds(Pose2d curPose) {
            // Find closest
            Distance smallestDist = Meters.of(99999.9);
            int closestIdx = 0;
            for (int pathIdx = 0; pathIdx < m_path.size(); pathIdx++) {
                Distance distToPose = Util.dist(m_path.get(pathIdx), curPose);
                if (smallestDist.gt(distToPose)) {
                    smallestDist = distToPose;
                    closestIdx = pathIdx;
                }
            }

            // If too far from target, go to the closest point
            Pose2d targetPose;
            if (smallestDist.gt(m_interpolationDistance.times(0.5))) {
                targetPose = m_path.get(closestIdx);
            } else {
                targetPose = m_path.get(Math.min(m_path.size() - 1, closestIdx + Math.max(1, (int) (m_lookAheadDistance.in(Meters) / m_interpolationDistance.in(Meters)))));
            }

            // 
            return new ChassisSpeeds(
                MetersPerSecond.of(m_xController.calculate(curPose.getMeasureX().in(Meters), targetPose.getMeasureX().in(Meters))),
                MetersPerSecond.of(m_yController.calculate(curPose.getMeasureY().in(Meters), targetPose.getMeasureY().in(Meters))),
                RadiansPerSecond.of(m_rotationController.calculate(curPose.getRotation().getRadians(), targetPose.getRotation().getRadians()))
            );
        }

        public boolean isFinished() {
            return m_xController.atSetpoint() &&
                   m_yController.atSetpoint() &&
                   m_rotationController.atSetpoint();
        }
    }
}
