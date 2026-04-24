package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.lib.motor.talonFX.SimpleTalon;

public class IndexerSim extends Indexer
{
  public SimpleTalon m_indexerMotor;
  public SimpleTalon m_kickerMotor;

  public DCMotorSim  m_indexerModel;
  public DCMotorSim  m_kickerModel;

  public IndexerSim()
  {
    m_inputs  = new Inputs();
    m_outputs = new Outputs();

    m_indexerMotor = new SimpleTalon(Constants.CanIds.IndexerMotorId,  Constants.Indexer.BeltConfig,   true); // is an X60
    m_kickerMotor  = new SimpleTalon(Constants.CanIds.KickerMotorId, Constants.Indexer.KickerConfig, false);

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
  public void setIndexers(AngularVelocity indexerVelocity, AngularVelocity kickerVelocity)
  {
    m_kickerMotor.setVelocity(kickerVelocity);
    m_indexerMotor.setVelocity(indexerVelocity);

    m_indexerModel.setInputVoltage(m_kickerMotor.getAppliedVoltage().in(Volts));
    m_indexerModel.update(0.02);
    
    m_kickerModel.setInputVoltage(m_indexerMotor.getAppliedVoltage().in(Volts));
    m_kickerModel.update(0.02);

    m_kickerMotor.simPeriodic(m_kickerModel.getAngularVelocity());
    m_indexerMotor.simPeriodic(m_indexerModel.getAngularVelocity());
  }

  @Override
  public void brakeIndexers()
  {
    m_indexerMotor.setVelocity(RotationsPerSecond.of(0.0));
    m_indexerModel.setInputVoltage(m_indexerMotor.getAppliedVoltage().in(Volts));
    m_indexerModel.update(0.02);
    m_indexerMotor.simPeriodic(m_indexerModel.getAngularVelocity());

    m_kickerMotor.setVelocity(RotationsPerSecond.of(0.0));
    m_kickerModel.setInputVoltage(m_kickerMotor.getAppliedVoltage().in(Volts));
    m_kickerModel.update(0.02);
    m_kickerMotor.simPeriodic(m_kickerModel.getAngularVelocity());
  }

  @Override
  public Pair<AngularVelocity, AngularVelocity> getVelocities()
  {
    return new Pair<AngularVelocity, AngularVelocity>(m_indexerMotor.getVelocity(), m_kickerMotor.getVelocity());
  }
}
