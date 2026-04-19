package frc.robot.subsystems.belts;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.lib.motor.talonFX.SimpleTalon;

public class BeltsTalonFX extends Belts
{
    public SimpleTalon m_beltMotor;
    public SimpleTalon m_kickerMotor;

    public BeltsTalonFX() {
        m_inputs = new Inputs();
        m_outputs = new Outputs();

        m_beltMotor   = new SimpleTalon(Constants.CanIds.BeltsMotorId, Constants.Indexer.BeltConfig);
        m_kickerMotor = new SimpleTalon(Constants.CanIds.KickerMotorId, Constants.Indexer.KickerConfig);
    }

    @Override
    public void setIndexers(AngularVelocity beltsVelocity, AngularVelocity kickerVelocity)
    {
        m_beltMotor.setVelocity(beltsVelocity);
        m_kickerMotor.setVelocity(kickerVelocity);
    }

    @Override
    public void brakeIndexers()
    {
        m_beltMotor.brake();
        m_kickerMotor.brake();
    }

    @Override
    public Pair<AngularVelocity, AngularVelocity> getVelocities()
    {
        return new Pair<AngularVelocity, AngularVelocity>(m_beltMotor.getVelocity(), m_kickerMotor.getVelocity());
    }
}
