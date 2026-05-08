package frc.robot.subsystems.gyro;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroSim implements GyroIO
{
  private final Supplier<Rotation2d> m_gyroSim;
  private Rotation2d                 m_lastMeasurement;

  public GyroSim(Supplier<Rotation2d> gyroSim) {

    m_gyroSim = gyroSim;
    m_lastMeasurement = m_gyroSim.get();
  }

  @Override
  public Rotation2d getGyroRotation() {

    return m_gyroSim.get();
  }

  @Override
  public AngularVelocity getGyroAngularVelocity() {

    AngularVelocity velocity = RadiansPerSecond.of(m_gyroSim.get().minus(m_lastMeasurement).div(0.02).getMeasure().in(Radians));
    m_lastMeasurement = m_gyroSim.get();
    return velocity;
  }
  
  @Override
  public void reset() {
    
  }
}
