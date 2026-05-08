package frc.robot.subsystems.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public interface GyroIO {

  public Rotation2d      getGyroRotation();
  public AngularVelocity getGyroAngularVelocity();  

  public void reset();
}
