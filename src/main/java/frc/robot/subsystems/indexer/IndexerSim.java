package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.lib.motor.io.TalonIO;

public class IndexerSim implements IndexerIO
{
  public TalonIO m_beltMotor;
  public TalonIO m_kickerMotor;

  public DCMotorSim  m_indexerModel;
  public DCMotorSim  m_kickerModel;

  public IndexerSim() {

    m_beltMotor   = new TalonIO(Constants.CanIds.IndexerMotorId,  Constants.Indexer.BeltConfig,   true); // is an X60
    m_kickerMotor = new TalonIO(Constants.CanIds.KickerMotorId, Constants.Indexer.KickerConfig, false);

    m_indexerModel = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
        DCMotor.getKrakenX60(1), 0.021, 1.0 // MOI from CAD
      ),
      DCMotor.getKrakenX60(1)
    );

    m_kickerModel = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
        DCMotor.getKrakenX44(1), 0.021, 1.0 // MOI from CAD
      ),
      DCMotor.getKrakenX44(1)
    );
  }

  @Override
  public void setBelts(AngularVelocity indexerVelocity) {

    m_beltMotor.setVelocity(indexerVelocity);
    m_indexerModel.setInputVoltage(m_beltMotor.getAppliedVoltage().in(Volts));
    m_indexerModel.update(0.02);
    m_beltMotor.simPeriodic(m_indexerModel.getAngularVelocity());
  }

  @Override
  public void setKicker(AngularVelocity kickerVelocity) {

    m_kickerMotor.setVelocity(kickerVelocity);
    m_kickerModel.setInputVoltage(m_kickerMotor.getAppliedVoltage().in(Volts));
    m_kickerModel.update(0.02);
    m_kickerMotor.simPeriodic(m_kickerModel.getAngularVelocity());
  }

  @Override
  public void brakeMotors() {
    setBelts(RotationsPerSecond.of(0.0));
    setKicker(RotationsPerSecond.of(0.0));
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
