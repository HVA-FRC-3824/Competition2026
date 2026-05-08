package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import org.ironmaple.simulation.IntakeSimulation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.lib.Module;
import frc.robot.lib.Module.Logged;

public class Intake extends Module<Intake.Outputs> {
  
  private Angle m_desiredAngle;

  private IntakeIO m_io;

  public Intake(IntakeIO io) {

    m_io = io;
    
    m_outputs = Outputs.zeroed();
  }

  public Command starting() {
    return runOnce(() -> m_desiredAngle = Rotations.of(9999.9));
  }  

  public Command stowed() {

    return runOnce(() -> {
      m_io.setPos(Constants.Intake.IntakeStowedTurns);
      m_desiredAngle = Constants.Intake.IntakeStowedTurns;
    });
  }

  public Command deployed() {
    
    return runOnce(() -> {
      m_io.setPos(Constants.Intake.IntakeDeployedTurns);
      m_desiredAngle = Constants.Intake.IntakeDeployedTurns;
    });
  }

  public Command alligator() {
    return stowed()
      .andThen(new WaitCommand(Seconds.of(0.4)))
      .andThen(deployed())
      .andThen(new WaitCommand(Seconds.of(0.4)))
      .repeatedly();
  }

  @Override
  public void updateOutputs()
  {
    m_outputs = new Intake.Outputs(m_desiredAngle, m_io.getPos());
  }

  public IntakeSimulation getSimIntake() {
    return null;
  }

  public static record Outputs(
    Angle desiredPos, Angle measuredPos
  ) implements Logged {

    public static Outputs zeroed() {

      return new Outputs(
        Rotations.of(0.0),
        Rotations.of(0.0));
    }

    @Override
    public void log() {

      Logger.recordOutput("Intake/Measured Pos", measuredPos);
      Logger.recordOutput("Intake/Desired Pos",  desiredPos);
    }
  }
}
