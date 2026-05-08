package frc.robot.lib;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public record VisionData(
    Pose2d visionMeasurement, 
    double timestampSeconds, 
    Matrix<N3, N1> stdDevs,
    int target
) {
  
}
