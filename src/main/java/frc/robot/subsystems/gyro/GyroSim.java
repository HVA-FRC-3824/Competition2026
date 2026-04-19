package frc.robot.subsystems.gyro;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroSim extends Gyro
{
    private final GyroSimulation m_gyroSim;

    public GyroSim(GyroSimulation gyroSim)
    {
        m_inputs = new Inputs();
        m_outputs = new Outputs();

        m_gyroSim = gyroSim;
    }

    @Override
    public Rotation2d getGyroRotation() 
    {
        return m_gyroSim.getGyroReading();
    }

    @Override
    public AngularVelocity getGyroAngularVelocity() 
    {
        return m_gyroSim.getMeasuredAngularVelocity();
    }
    
    @Override
    public void resetGyroAngle() 
    {
        m_gyroSim.setRotation(new Rotation2d(0.0));
    }
}
