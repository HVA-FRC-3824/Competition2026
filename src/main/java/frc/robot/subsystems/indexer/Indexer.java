package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.lib.Logged;
import frc.robot.lib.Module;

public abstract class Indexer extends Module<Indexer.Inputs, Indexer.Outputs>
{
  public enum State
  {
    On,
    Off,
    Backwards
  }

  public static class Inputs
  {
    public State m_state = State.Off;

    public Inputs(State state)
    {
      m_state = state;
    }

    public Inputs() {
      
    }
  }

  public static class Outputs extends Logged
  {
    public State       m_state;
    public AngularVelocity m_desiredIndexerVelocity;
    public AngularVelocity m_desiredKickerVelocity;
    public AngularVelocity m_measuredIndexerVelocity;
    public AngularVelocity m_measuredKickerVelocity;

    public Outputs()
    {
      m_state = State.Off;
      m_desiredIndexerVelocity  = RotationsPerSecond.of(0.0);
      m_desiredKickerVelocity   = RotationsPerSecond.of(0.0);
      m_measuredIndexerVelocity = RotationsPerSecond.of(0.0);
      m_measuredKickerVelocity  = RotationsPerSecond.of(0.0);
    }

    public Outputs(State state, 
             AngularVelocity desiredIndexerVelocity, 
             AngularVelocity desiredKickerVelocity, 
             AngularVelocity measuredIndexerVelocity, 
             AngularVelocity measuredKickerVelocity)
    {
      m_state = state;
      m_desiredIndexerVelocity  = desiredIndexerVelocity;
      m_desiredKickerVelocity   = desiredKickerVelocity;
      m_measuredIndexerVelocity = measuredIndexerVelocity;
      m_measuredKickerVelocity  = measuredKickerVelocity;
      
    }

    public void log() {
      Logger.recordOutput("Indexer/State", m_state);
      Logger.recordOutput("Indexer/Desired Indexer Velocity",  m_desiredIndexerVelocity);
      Logger.recordOutput("Indexer/Desired Kicker Velocity",   m_desiredKickerVelocity);
      Logger.recordOutput("Indexer/Measured Indexer Velocity", m_measuredIndexerVelocity);
      Logger.recordOutput("Indexer/Measured Kicker Velocity",  m_measuredKickerVelocity);
    }
  }

  Pair<AngularVelocity, AngularVelocity> m_desiredVelocities 
    = new Pair<>(RotationsPerSecond.of(0.0), RotationsPerSecond.of(0.0));

  @Override
  protected void updateHardwareInputs() {
    switch (m_inputs.m_state) {
      case Off:
        brakeIndexers();
        m_desiredVelocities = new Pair<>(
          RotationsPerSecond.of(0.0), 
          RotationsPerSecond.of(0.0));
        break;
        
      case On:
        setIndexers(Constants.Indexer.BeltTurnsPerSec, Constants.Indexer.KickerWheelTurnsPerSec);
        m_desiredVelocities = new Pair<>(
          Constants.Indexer.BeltTurnsPerSec, 
          Constants.Indexer.KickerWheelTurnsPerSec);
        break;

      case Backwards:
        setIndexers(Constants.Indexer.BeltTurnsPerSec.times(-1), Constants.Indexer.KickerWheelTurnsPerSec.times(-1));
        m_desiredVelocities = new Pair<>(
          Constants.Indexer.BeltTurnsPerSec.times(-1), 
          Constants.Indexer.KickerWheelTurnsPerSec.times(-1));
        break;

      default:
        break;
    }
  }

  @Override
  protected void updateOutputs() {
    Pair<AngularVelocity, AngularVelocity> velocities = getVelocities();
    
    m_outputs = new Outputs(m_inputs.m_state, 
                m_desiredVelocities.getFirst(), 
                m_desiredVelocities.getSecond(), 
                velocities.getFirst(), 
                velocities.getSecond());
  }

  protected abstract void setIndexers(AngularVelocity indexerVelocity, AngularVelocity kickerVelocity); 

  protected abstract void brakeIndexers();

  protected abstract Pair<AngularVelocity, AngularVelocity> getVelocities();
}
