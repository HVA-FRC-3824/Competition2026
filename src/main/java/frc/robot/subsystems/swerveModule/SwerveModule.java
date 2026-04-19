package frc.robot.subsystems.swerveModule;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.lib.Logged;
import frc.robot.lib.Module;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public abstract class SwerveModule extends Module<SwerveModule.Inputs, SwerveModule.Outputs>
{
    public enum State
    {
        Driving
    }

    public static class Inputs
    {
        public SwerveModuleState m_moduleState = new SwerveModuleState();
        public State             m_state       = State.Driving;

        public Inputs(SwerveModuleState moduleState, State state)
        {
            m_moduleState = moduleState;
            m_state = state;
        }

        public Inputs()
        {

        }
    }

    public static class Outputs extends Logged
    {
        public SwerveModuleState    m_measuredSwerveModuleState;
        public SwerveModuleState    m_desiredSwerveModuleState;
        public SwerveModulePosition m_swerveModulePosition;
        public State                m_state;

        public String               m_moduleName;

        public Outputs()
        {
            m_measuredSwerveModuleState = new SwerveModuleState();
            m_desiredSwerveModuleState  = new SwerveModuleState();
            m_swerveModulePosition      = new SwerveModulePosition();
            m_state                     = State.Driving;
            m_moduleName                = "";
        }

        public Outputs(SwerveModuleState measuredSwerveModuleState, 
                       SwerveModuleState desiredSwerveModuleState,
                       SwerveModulePosition swerveModulePosition, 
                       State state)
        {
            m_measuredSwerveModuleState = measuredSwerveModuleState;
            m_desiredSwerveModuleState  = desiredSwerveModuleState;
            m_swerveModulePosition      = swerveModulePosition; // Not logged on ds...
            m_state                     = state;
        }
        
        public void log() {
            Logger.recordOutput("" + m_moduleName + "/Measured drive ", m_measuredSwerveModuleState.speedMetersPerSecond);
            Logger.recordOutput("" + m_moduleName + "/Measured angle ", m_measuredSwerveModuleState.angle);
            Logger.recordOutput("" + m_moduleName + "/Desired drive ", m_desiredSwerveModuleState.speedMetersPerSecond);
            Logger.recordOutput("" + m_moduleName + "/Desired angle ", m_desiredSwerveModuleState.angle);
            Logger.recordOutput("" + m_moduleName + "/State ", m_state);
        }
    }

    @Override
    public void updateOutputs()
    {
        m_outputs = new Outputs(getState(), getState(), getPosition(), m_inputs.m_state);
    }

    @Override
    public void updateHardwareInputs() 
    {
        // Optimize state
        m_inputs.m_moduleState.optimize(getPosition().angle);

        // Set velocity
        AngularVelocity velocity = 
            RotationsPerSecond.of(m_inputs.m_moduleState.speedMetersPerSecond / Constants.Chassis.DriveMotorConversion);

        setPosition(m_inputs.m_moduleState.angle.getMeasure());
        setVelocity(velocity);
    }

    abstract protected void setPosition(Angle angle);
    abstract protected void setVelocity(AngularVelocity velocity);
    abstract protected SwerveModuleState getState();
    abstract protected SwerveModulePosition getPosition();
    abstract protected int getNum();

    abstract public void resetEncoders();
    abstract public void setWheelAngleToForward(Angle forwardAngleDeg);
}