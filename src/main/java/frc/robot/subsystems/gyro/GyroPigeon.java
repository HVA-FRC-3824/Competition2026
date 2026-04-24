package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;

public class GyroPigeon extends Gyro
{
  Pigeon2 m_gyro;

  public GyroPigeon() {
    m_inputs = new Inputs();
    m_outputs = new Outputs();

    m_gyro = new Pigeon2(Constants.CanIds.PigeonGyroId);
  }

  @Override
  public Rotation2d getGyroRotation() 
  {
    return m_gyro.getRotation2d();
  }

  @Override
  public AngularVelocity getGyroAngularVelocity() 
  {
    return m_gyro.getAngularVelocityZWorld().getValue();
  }

  @Override
  public void resetGyroAngle() 
  {
    m_gyro.reset();
  }
}
