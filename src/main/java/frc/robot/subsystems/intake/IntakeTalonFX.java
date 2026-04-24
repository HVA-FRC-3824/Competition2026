package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.lib.motor.talonFX.SimpleTalon;

public class IntakeTalonFX extends Intake
{
  public SimpleTalon m_motor;

  public IntakeTalonFX()
  {
    m_inputs = new Inputs();
    m_outputs = new Outputs();

    m_motor = new SimpleTalon(Constants.CanIds.IntakePositionFollowerMotorId, Constants.Intake.PivotConfig);
  }

  @Override
  public void setPos(Angle angle)
  {
    m_motor.setPosition(angle);
  }

  @Override
  public Angle getPos()
  {
    return m_motor.getPos();
  }
}
