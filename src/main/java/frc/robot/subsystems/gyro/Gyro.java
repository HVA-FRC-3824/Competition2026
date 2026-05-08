package frc.robot.subsystems.gyro;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.Module;
import frc.robot.lib.Module.Logged;

public class Gyro extends Module<Gyro.Outputs>
{

  GyroIO m_io;

  public Gyro(GyroIO io) {

    m_io = io;

    m_outputs = Outputs.zeroed();
  }

  @Override
  public void updateOutputs() {
    m_outputs = new Outputs(m_io.getGyroRotation().getMeasure(), m_io.getGyroAngularVelocity());
  }

  public Command reset() {

    return runOnce(m_io::reset);
  }

  public static class Inputs
  {
    public Inputs() {}
  }

  public static record Outputs(
    Angle heading, AngularVelocity headingVelocity
  ) implements Logged {

    public static Outputs zeroed() {
      return new Outputs(
        Degrees.of(0.0),
        DegreesPerSecond.of(0.0));
    }

    @Override
    public void log() 
    {
      Logger.recordOutput("Gyro/Heading", heading);
      Logger.recordOutput("Gyro/Angular Velocity", headingVelocity);
    }
  }
}
