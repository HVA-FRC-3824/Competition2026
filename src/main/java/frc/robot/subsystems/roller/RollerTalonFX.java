package frc.robot.subsystems.roller;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.lib.motor.talonFX.SimpleTalon;

public class RollerTalonFX extends Roller
{
  public SimpleTalon m_motor;

  public RollerTalonFX() {
    m_inputs = new Inputs();
    m_outputs = new Outputs();

    m_motor = new SimpleTalon(Constants.CanIds.FuelIntakeMotorId, Constants.Roller.RollerConfig);
  }

  @Override
  protected void setRoller(AngularVelocity velocity)
  {
    m_motor.setVelocity(velocity);
  }

  @Override
  protected void brakeRoller()
  {
    m_motor.brake();
  }

  @Override
  protected AngularVelocity getVelocity()
  {
    return m_motor.getVelocity();
  }
}
