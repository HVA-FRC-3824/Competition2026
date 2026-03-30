package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public class Gyro implements GyroIO
{
    Pigeon2 m_gyro;

    public Gyro(int id)
    {
        m_gyro = new Pigeon2(id);
    }

    @Override
    public Rotation2d getGyroRotation() 
    {
        return m_gyro.getRotation2d();
    }

    @Override
    public AngularVelocity getGyroAngularVelocity() 
    {
        return m_gyro.getAngularVelocityZWorld().getValue();
    }

    @Override
    public void resetGyroAngle() 
    {
        m_gyro.reset();
    }
}
