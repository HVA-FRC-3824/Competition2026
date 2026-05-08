package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.Module;
import frc.robot.lib.Module.Logged;

public class Roller extends Module<Roller.Outputs> {

  private RollerIO m_io;

  public Roller(RollerIO io) {

    m_io = io;

    m_outputs = Outputs.zeroed();
  }

  public Command off() {
    return runOnce(m_io::brakeRoller);
  }
  
  public Command on() {
    return runOnce(() -> m_io.setRoller(Constants.Roller.IntakeDriveTurnsPerSec));
  }

  public Command backwards() {
    return runOnce(() -> m_io.setRoller(Constants.Roller.IntakeDriveTurnsPerSec.times(-1.0)));
  }

  @Override
  public void updateOutputs() {
    m_outputs = new Outputs(m_io.getVelocity());
  }

  public static record Outputs(
    AngularVelocity measuredVelocity
  ) implements Logged {

    public static Outputs zeroed() {
      return new Outputs(
        RotationsPerSecond.of(0.0));
    }

    @Override
    public void log() {
      Logger.recordOutput("Roller/Measured Velocity",  measuredVelocity);
    }
  }
}
