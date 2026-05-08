package frc.robot.subsystems.flywheel;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.lib.motor.io.TalonIO;

public class FlywheelIOTalonFX implements FlywheelIO 
{
  TalonIO m_motor;
  TalonIO m_motorFollower;

  public FlywheelIOTalonFX()
  {
    m_motor     = new TalonIO(Constants.CanIds.FlywheelMotorId, Constants.Flywheel.Config);
    m_motorFollower = new TalonIO(Constants.CanIds.FlywheelFollowerMotorId, Constants.Flywheel.Config);

    m_motorFollower.follow(Constants.CanIds.FlywheelMotorId, true);
  }

  @Override
  public void setFlywheel(AngularVelocity velocity) {
    m_motor.setVelocity(velocity);
  }

  @Override
  public void stopFlywheel() {
    m_motor.brake();
  }

  @Override
  public AngularVelocity getMeasured() {
    return m_motor.getVelocity();
  }
}