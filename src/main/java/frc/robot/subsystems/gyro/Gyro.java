package frc.robot.subsystems.gyro;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.lib.Logged;
import frc.robot.lib.Module;

public abstract class Gyro extends Module<Gyro.Inputs, Gyro.Outputs>
{
  public class Inputs
  {
    public Inputs()
    {

    }
  }

  public class Outputs extends Logged
  {
    public AngularVelocity m_headingVelocity;
    public Angle       m_heading;

    public Outputs()
    {
      m_heading     = Degrees.of(0.0);
      m_headingVelocity = DegreesPerSecond.of(0.0);
    }

    public Outputs(Angle heading, AngularVelocity headingVelocity)
    {
      m_heading     = heading;
      m_headingVelocity = headingVelocity;
    }

    public void log() 
    {
      Logger.recordOutput("Gyro/Heading", m_heading);
      Logger.recordOutput("Gyro/Angular Velocity",  m_headingVelocity);
    }
  }

  @Override
  protected void updateOutputs() {
    m_outputs = new Outputs(getGyroRotation().getMeasure(), getGyroAngularVelocity());
  }

  @Override
  protected void updateHardwareInputs() {}

  abstract protected Rotation2d getGyroRotation();
  abstract protected AngularVelocity getGyroAngularVelocity();  
  
  // This one is kinda an outlier to the architecture
  abstract public void resetGyroAngle();
}
