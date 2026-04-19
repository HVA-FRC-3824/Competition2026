package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.lib.Logged;
import frc.robot.lib.Module;

public abstract class Roller extends Module<Roller.Inputs, Roller.Outputs>
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

        public Inputs(State state) {
            m_state = state;
        }

        public Inputs()
        {

        }
    }

    public static class Outputs extends Logged
    {
        State           m_state;
        AngularVelocity m_measuredVelocity;

        public Outputs()
        {
            m_state            = State.Off;
            m_measuredVelocity = RotationsPerSecond.of(0.0);
        }

        public Outputs(State state, AngularVelocity measuredVelocity)
        {
            m_state            = state;
            m_measuredVelocity = measuredVelocity;
        }

        public void log() {
            Logger.recordOutput("Roller/State",              m_state);
            Logger.recordOutput("Roller/Measured Velocity",  m_measuredVelocity);
        }
    }

    @Override
    public void updateHardwareInputs() {
        switch (m_inputs.m_state) {
            case Off:
                brakeRoller();
                break;
            case On:
                setRoller(Constants.Roller.IntakeDriveTurnsPerSec);
                break;
            case Backwards:
                setRoller(Constants.Roller.IntakeDriveTurnsPerSec.times(-1.0));
                break;
        }
    }

    @Override
    public void updateOutputs() {
        m_outputs = new Outputs(m_inputs.m_state, getVelocity());
    }

    abstract protected void setRoller(AngularVelocity velocity); 
    abstract protected void brakeRoller();
    abstract protected AngularVelocity getVelocity();
}
