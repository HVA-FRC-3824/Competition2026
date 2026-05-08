package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.lib.Module;
import frc.robot.lib.Module.Logged;
import frc.robot.subsystems.swerve.Swerve;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;

public class Flywheel extends Module<Flywheel.Outputs>
{
  private AngularVelocity m_lastInput = RotationsPerSecond.of(0.0);
  
  private int m_offsetIndex = 0;

  FlywheelIO m_io;

  SensorData m_data = new SensorData(Inches.of(0.0));

  Swerve.AimTarget m_target = Swerve.AimTarget.Score;

  public Flywheel(FlywheelIO io) {
    
    m_io = io;

    m_outputs = Outputs.zeroed();
  }
  
	public void updateSensorData(SensorData data) {
		
		m_data = data;
	}

  public Swerve.AimTarget getTarget() {
    return m_target;
  }

  public Command off() {
    
    return runOnce(() -> {
      m_io.stopFlywheel();
      m_lastInput = RotationsPerSecond.of(0.0);
    });
  }

  public Command auto(Swerve.AimTarget target) {

    return run(() -> {
      m_target = target;

      AngularVelocity velocityFromDist = RotationsPerSecond.of(0.166667 * m_data.distFromTarget().in(Meters) + 30.33333);
      m_io.setFlywheel(velocityFromDist);
      m_lastInput = velocityFromDist;
    });
  }

  public Command manual(AngularVelocity velocity) {

    return runOnce(() -> {
      m_io.setFlywheel(velocity);
      m_lastInput = velocity;
    });
  }

  public Command set(Setpoints setpoint) {

    return runOnce(() -> {
      m_io.setFlywheel(setpoint.m_velocity);
      m_lastInput = setpoint.m_velocity;
    });
  }

  // Update logging struct (m_outputs)
  @Override
  public void updateOutputs()
  {
    m_outputs = new Outputs(
      getReference(),
      m_io.getMeasured(),
      isSpunUp()
    );
  }

  protected boolean isSpunUp() {
    return getReference().isNear(m_io.getMeasured(), Constants.Flywheel.SpunUpTolerance)
           && 
           getReference().in(RotationsPerSecond) != 0.0;
  }

  protected AngularVelocity getReference() {
    return m_lastInput;
  }

  public record SensorData(
    Distance distFromTarget
  ) {

  }

  public enum Setpoints {
    Backwards(Constants.Flywheel.CloseSpeed.times(-1.0)),
    Low(Constants.Flywheel.CloseSpeed),
    Mid(Constants.Flywheel.MiddleSpeed),
    Neutral(Constants.Flywheel.NeutralPassSpeed),
    Field(Constants.Flywheel.FieldPassSpeed);

    AngularVelocity m_velocity; 

    private Setpoints(AngularVelocity vel) {
      m_velocity = vel;
    }
  }

  public static record Outputs(
    AngularVelocity desiredTPS, AngularVelocity measuredTPS, boolean isSpunUp
  ) implements Logged {

    public static Outputs zeroed()
    {
      return new Outputs(
        RotationsPerSecond.of(0.0),
        RotationsPerSecond.of(0.0),
        false);
    }

    @Override
    public void log() {
      Logger.recordOutput("Flywheel/Desired turns per sec",  desiredTPS);
      Logger.recordOutput("Flywheel/Measured turns per sec", measuredTPS);
      Logger.recordOutput("Flywheel/Is Spun Up",             isSpunUp);
    }
  }
}
