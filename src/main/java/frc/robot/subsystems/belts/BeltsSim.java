package frc.robot.subsystems.belts;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.lib.motor.talonFX.SimpleTalon;

public class BeltsSim extends Belts
{
    public SimpleTalon m_beltsMotor;
    public SimpleTalon m_kickerMotor;

    public DCMotorSim  m_beltsModel;
    public DCMotorSim  m_kickerModel;

    public BeltsSim()
    {
        m_inputs = new Inputs();
        m_outputs = new Outputs();

        m_beltsMotor  = new SimpleTalon(Constants.CanIds.BeltsMotorId, Constants.Indexer.BeltConfig, true); // is an X60
        m_kickerMotor = new SimpleTalon(Constants.CanIds.KickerMotorId, Constants.Indexer.KickerConfig, true);

        m_beltsModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), 0.012292036, 1.0 // MOI from CAD
            ),
            DCMotor.getKrakenX60(1)
        );

        m_kickerModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), 0.0382822415, 1.0 // MOI from CAD
            ),
            DCMotor.getKrakenX60(1)
        );
    }

    @Override
    public void setIndexers(AngularVelocity beltsVelocity, AngularVelocity kickerVelocity)
    {
        m_kickerMotor.setVelocity(kickerVelocity);
        m_beltsMotor.setVelocity(beltsVelocity);

        m_beltsModel.setInputVoltage(m_kickerMotor.getAppliedVoltage().in(Volts));
        m_beltsModel.update(0.02);
        
        m_kickerModel.setInputVoltage(m_beltsMotor.getAppliedVoltage().in(Volts));
        m_kickerModel.update(0.02);

        m_kickerMotor.simPeriodic(m_kickerModel.getAngularVelocity());
        m_beltsMotor.simPeriodic(m_beltsModel.getAngularVelocity());
    }

    @Override
    public void brakeIndexers()
    {
        m_beltsMotor.brake();
        m_kickerMotor.brake();
        
        m_beltsModel.setInputVoltage(m_beltsMotor.getAppliedVoltage().in(Volts));
        m_beltsModel.update(0.02);
        
        m_kickerModel.setInputVoltage(m_kickerMotor.getAppliedVoltage().in(Volts));
        m_kickerModel.update(0.02);

        m_beltsMotor.simPeriodic(m_beltsModel.getAngularVelocity());
        m_kickerMotor.simPeriodic(m_kickerModel.getAngularVelocity());
    }

    @Override
    public Pair<AngularVelocity, AngularVelocity> getVelocities()
    {
        return new Pair<AngularVelocity, AngularVelocity>(m_beltsMotor.getVelocity(), m_kickerMotor.getVelocity());
    }
}
