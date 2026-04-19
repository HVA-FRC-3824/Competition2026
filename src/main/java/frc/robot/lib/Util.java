package frc.robot.lib;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
}