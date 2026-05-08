package frc.robot.subsystems.roller;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.lib.motor.io.TalonIO;

public class RollerTalonFX implements RollerIO
{
  public TalonIO m_motor;

  public RollerTalonFX() {

    m_motor = new TalonIO(Constants.CanIds.FuelIntakeMotorId, Constants.Roller.RollerConfig);
  }

  @Override
  public void setRoller(AngularVelocity velocity)
  {
    m_motor.setVelocity(velocity);
  }

  @Override
  public void brakeRoller()
  {
    m_motor.brake();
  }

  @Override
  public AngularVelocity getVelocity()
  {
    return m_motor.getVelocity();
  }
}
