package frc.robot.subsystems.gyro;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroIOSim implements GyroIO
{
    private final GyroSimulation m_gyroSim;

    public GyroIOSim(GyroSimulation gyroSim)
    {
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
