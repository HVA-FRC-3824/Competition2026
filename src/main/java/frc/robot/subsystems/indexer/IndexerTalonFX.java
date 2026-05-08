package frc.robot.subsystems.indexer;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.lib.motor.io.TalonIO;

public class IndexerTalonFX implements IndexerIO
{
  public TalonIO m_beltMotor;
  public TalonIO m_kickerMotor;

  public IndexerTalonFX() {

    m_beltMotor   = new TalonIO(Constants.CanIds.IndexerMotorId, Constants.Indexer.BeltConfig);
    m_kickerMotor = new TalonIO(Constants.CanIds.KickerMotorId, Constants.Indexer.KickerConfig);
  }

  @Override
  public void setBelts(AngularVelocity indexerVelocity) {

    m_beltMotor.setVelocity(indexerVelocity);
  }

  @Override
  public void setKicker(AngularVelocity kickerVelocity) {

    m_kickerMotor.setVelocity(kickerVelocity);
  }

  @Override
  public void brakeMotors() {
    m_beltMotor.brake();
    m_kickerMotor.brake();
  }

  @Override
  public AngularVelocity getIndexerVelocity() {
    return m_beltMotor.getVelocity();
  }

  @Override
  public AngularVelocity getKickerVelocity() {
    return m_kickerMotor.getVelocity();

  }
}
