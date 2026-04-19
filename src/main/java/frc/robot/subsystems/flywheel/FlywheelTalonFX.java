package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.lib.motor.talonFX.SimpleTalon;

public class FlywheelTalonFX extends Flywheel 
{
    SimpleTalon m_motor;
    SimpleTalon m_motorFollower;

    AngularVelocity m_lastInput;

    public FlywheelTalonFX()
    {
        m_inputs = new Inputs();
        m_outputs = new Outputs();

        m_motor         = new SimpleTalon(Constants.CanIds.FlywheelMotorId, Constants.Flywheel.Config);
        m_motorFollower = new SimpleTalon(Constants.CanIds.FlywheelFollowerMotorId, Constants.Flywheel.Config);

        m_motorFollower.follow(Constants.CanIds.FlywheelMotorId, true);

        m_lastInput = RotationsPerSecond.of(0.0);
    }

    protected void setFlywheel(AngularVelocity velocity)
    {
        m_lastInput = velocity;
        m_motor.setVelocity(velocity);
    }

    protected void stopFlywheel()
    {
        m_motor.brake();
    }

    protected AngularVelocity getReference()
    {
        return m_lastInput;
    }

    protected AngularVelocity getMeasured()
    {
        return m_motor.getVelocity();
    }
}