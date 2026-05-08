package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.Module;
import frc.robot.lib.Module.Logged;

public class Indexer extends Module<Indexer.Outputs>
{
  IndexerIO m_io;

  public Indexer(IndexerIO io) {
    
    m_io = io;

    m_outputs = Outputs.zeroed();
  }
  
  private AngularVelocity m_desiredKicker = RotationsPerSecond.of(0.0);
  private AngularVelocity m_desiredIndexer = RotationsPerSecond.of(0.0);

  public Command off() {
    
    return runOnce(() -> {
      m_io.brakeMotors();
      m_desiredIndexer = RotationsPerSecond.of(0.0); 
      m_desiredKicker = RotationsPerSecond.of(0.0);
    });
  }

  public Command on() {

    return runOnce(() -> {
      m_io.setBelts(Constants.Indexer.BeltTurnsPerSec);
      m_io.setKicker(Constants.Indexer.KickerWheelTurnsPerSec);
      m_desiredIndexer = Constants.Indexer.BeltTurnsPerSec;
      m_desiredKicker = Constants.Indexer.KickerWheelTurnsPerSec;
    });
  }

  public Command backwards() {

    return runOnce(() -> {
      m_io.setBelts(Constants.Indexer.BeltTurnsPerSec.times(-1.0));
      m_io.setKicker( Constants.Indexer.KickerWheelTurnsPerSec.times(-1.0));
      m_desiredIndexer = Constants.Indexer.BeltTurnsPerSec.times(-1.0);
      m_desiredKicker  = Constants.Indexer.KickerWheelTurnsPerSec.times(-1.0);
    });
  }

  @Override
  public void updateOutputs() {    
    m_outputs = new Outputs( 
                m_desiredIndexer, 
                m_desiredKicker, 
                m_io.getIndexerVelocity(), 
                m_io.getKickerVelocity());
  }

  public static record Outputs(
    AngularVelocity desiredIndexerVelocity,
    AngularVelocity desiredKickerVelocity,
    AngularVelocity measuredIndexerVelocity,
    AngularVelocity measuredKickerVelocity
  ) implements Logged {

    public static Outputs zeroed() {
      return new Outputs(
        RotationsPerSecond.of(0.0),
        RotationsPerSecond.of(0.0),
        RotationsPerSecond.of(0.0),
        RotationsPerSecond.of(0.0));
    }

    @Override
    public void log() {
      Logger.recordOutput("Indexer/Desired Indexer Velocity",  desiredIndexerVelocity);
      Logger.recordOutput("Indexer/Desired Kicker Velocity",   desiredKickerVelocity);
      Logger.recordOutput("Indexer/Measured Indexer Velocity", measuredIndexerVelocity);
      Logger.recordOutput("Indexer/Measured Kicker Velocity",  measuredKickerVelocity);
    }
  }
}
