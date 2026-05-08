package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.lib.motor.io.TalonIO;

public class IntakeTalonFX implements IntakeIO
{
  public TalonIO m_motor;
  public TalonIO m_motorFollower;

  public IntakeTalonFX() {

    m_motor         = new TalonIO(Constants.CanIds.IntakePositionLeaderMotorId, Constants.Intake.PivotConfig);
    m_motorFollower = new TalonIO(Constants.CanIds.IntakePositionFollowerMotorId, Constants.Intake.PivotConfig);

    m_motorFollower.follow(Constants.CanIds.IntakePositionLeaderMotorId, true);
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
