package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import org.ironmaple.simulation.IntakeSimulation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.lib.Logged;
import frc.robot.lib.Module;

public abstract class Intake extends Module<Intake.Inputs, Intake.Outputs>
{
    public enum State
    {
        Starting,
        Stowed,
        Deployed,
        Alligator
    }

    public static class Inputs
    {
        public State m_state = State.Starting;
        public boolean m_rollersOn = false;

        public Inputs(State state) {
            m_state = state;
        }

        public Inputs(State state, boolean rollersOn) {
            m_state = state;
            m_rollersOn = rollersOn;
        }

        public Inputs() {
        }
    }

    public static class Outputs extends Logged
    {
        public State m_state;
        public Angle m_desiredPos;
        public Angle m_measuredPos;

        public Outputs() {
            m_state       = State.Starting;
            m_desiredPos  = Rotations.of(0.0);
            m_measuredPos = Rotations.of(0.0);
        }

        public Outputs(State state, Angle desiredPos, Angle measuredPos) {
            m_state       = state;
            m_measuredPos = measuredPos;
            m_desiredPos  = desiredPos;
        }

        public void log() 
        {
            Logger.recordOutput("Intake/State",  m_state);
            Logger.recordOutput("Intake/Measured Pos", m_measuredPos);
            Logger.recordOutput("Intake/Desired Pos",  m_desiredPos);
        }
    }

    @Override
    public void updateHardwareInputs()
    {
        switch (m_inputs.m_state)
        {
            case Starting:
                break;
            case Stowed:
                setPos(Constants.Intake.IntakeStowedTurns);
                break;
            case Deployed:
                setPos(Constants.Intake.IntakeDeployedTurns);
                break;
            case Alligator:
                double time = Timer.getTimestamp();
                time = (time - (double) ((int) time)); // thx CSA
                if (time > 0.8)
                {
                    setPos(Constants.Intake.IntakeStowedTurns);
                }
                else if (time > 0.55)
                {
                    setPos(Constants.Intake.IntakeDeployedTurns);
                }
                else if (time > 0.4)
                {
                    setPos(Constants.Intake.IntakeStowedTurns);
                }
                else if (time > 0.25)
                {
                    setPos(Constants.Intake.IntakeDeployedTurns);
                }
                else
                {
                    setPos(Constants.Intake.IntakeStowedTurns);
                }
                break;
        }
    }

    @Override
    public void updateOutputs()
    {
        m_outputs = new Intake.Outputs(m_inputs.m_state, getPos(), getReference());
    }

    public IntakeSimulation getSimIntake() {
        return null;
    }

    abstract protected void setPos(Angle angle); 

    abstract protected Angle getPos();
    
    abstract protected Angle getReference();
}
