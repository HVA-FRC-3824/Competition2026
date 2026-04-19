package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.lib.motor.talonFX.SimpleTalon;

public class RollerSim extends Roller
{
    public SimpleTalon m_motor;

    public DCMotorSim  m_motorModel;

    public RollerSim()
    {
        m_inputs = new Inputs();
        m_outputs = new Outputs();

        m_motor = new SimpleTalon(Constants.CanIds.FuelIntakeMotorId, Constants.Roller.RollerConfig, true); // is an X60

        m_motorModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), 0.01102666212, 1.0 // MOI from CAD
            ),
            DCMotor.getKrakenX60(1)
        );
    }

    @Override
    protected void setRoller(AngularVelocity velocity)
    {
        m_motor.setVelocity(velocity);

        m_motorModel.setInputVoltage(m_motor.getAppliedVoltage().in(Volts));
        m_motorModel.update(0.02);

        m_motor.simPeriodic(m_motorModel.getAngularVelocity());
    }

    @Override
    protected void brakeRoller()
    {
        m_motor.brake();
        
        m_motorModel.setInputVoltage(m_motor.getAppliedVoltage().in(Volts));
        m_motorModel.update(0.02);

        m_motor.simPeriodic(m_motorModel.getAngularVelocity());
    }

    @Override
    protected AngularVelocity getVelocity()
    {
        return m_motor.getVelocity();
    }
}
