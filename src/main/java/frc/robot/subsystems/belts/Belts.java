package frc.robot.subsystems.belts;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.lib.Logged;
import frc.robot.lib.Module;

public abstract class Belts extends Module<Belts.Inputs, Belts.Outputs>
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
        public State           m_state;
        public AngularVelocity m_beltsVelocity;
        public AngularVelocity m_kickerVelocity;

        public Outputs()
        {
            m_state          = State.Off;
            m_beltsVelocity  = RotationsPerSecond.of(0.0);
            m_kickerVelocity = RotationsPerSecond.of(0.0);
        }

        public Outputs(State state, AngularVelocity beltsVelocity, AngularVelocity kickerVelocity)
        {
            m_state          = state;
            m_beltsVelocity  = beltsVelocity;
            m_kickerVelocity = kickerVelocity;
        }

        public void log() {
            Logger.recordOutput("Belts/State",           m_state);
            Logger.recordOutput("Belts/Belts Velocity",  m_beltsVelocity);
            Logger.recordOutput("Belts/Kicker Velocity", m_kickerVelocity);
        }
    }

    @Override
    public void updateHardwareInputs() {
        switch (m_inputs.m_state) {
            case Off:
                brakeIndexers();
            case On:
                setIndexers(Constants.Indexer.BeltTurnsPerSec, Constants.Indexer.KickerWheelTurnsPerSec);
            case Backwards:
                setIndexers(Constants.Indexer.BeltTurnsPerSec.times(-1), Constants.Indexer.KickerWheelTurnsPerSec.times(-1));
            default:
                break;
        }
    }

    @Override
    public void updateOutputs() {
        Pair<AngularVelocity, AngularVelocity> velocities = getVelocities();
        
        m_outputs = new Outputs(m_inputs.m_state, velocities.getFirst(), velocities.getSecond());
    }

    protected abstract void setIndexers(AngularVelocity beltsVelocity, AngularVelocity kickerVelocity); 

    protected abstract void brakeIndexers();

    protected abstract Pair<AngularVelocity, AngularVelocity> getVelocities();
}
