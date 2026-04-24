package frc.robot.lib;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class Util {
  public static Distance dist(Pose2d pose1, Pose2d pose2) {
    return Meters.of(
      Math.sqrt(Math.pow(pose1.getMeasureX().in(Meters) - pose2.getMeasureX().in(Meters), 2) + 
            Math.pow(pose1.getMeasureY().in(Meters) - pose2.getMeasureY().in(Meters), 2)));
  }

  public static Distance dist(Translation2d pose1, Translation2d pose2) {
    return Meters.of(
      Math.sqrt(Math.pow(pose1.getMeasureX().in(Meters) - pose2.getMeasureX().in(Meters), 2) + 
            Math.pow(pose1.getMeasureY().in(Meters) - pose2.getMeasureY().in(Meters), 2)));
  }

  public static record Vector(
    double x,
    double y,
    double magnitude,
    Angle theta
  ) {
    public static Vector FromCoords(double x, double y) {
      return new Vector(
        x, y, 
        Math.sqrt(Math.pow(x,2) + Math.pow(y,2)), Radians.of(Math.atan2(x, y))
      );
    }
    
    public static Vector FromMagTheta(double magnitude, Angle theta) {
      return new Vector(
        Math.cos(theta.in(Radians)) * magnitude, Math.sin(theta.in(Radians)) * magnitude,
        magnitude, theta);
    }

    public static double dot(Vector vec1, Vector vec2) {
      return vec1.dot(vec2);
    }

    public double dot(Vector vec) {
      return (x * vec.x()) + (y * vec.y());
    }
    
    public Vector RotateBy(double theta) {
      return Vector.FromCoords(
        x * Math.cos(theta) + y * Math.sin(theta),
        y * Math.cos(theta) - x * Math.sin(theta));
    }
    
    public Vector times(double scalar) {
      return Vector.FromCoords(
        x * scalar,
        y * scalar
      );
    }
  };
}