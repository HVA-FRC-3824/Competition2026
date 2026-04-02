package frc.robot.subsystems.tower;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.Follower;

import frc.robot.Constants;
import frc.robot.lib.TalonFXConfig;

public class Tower implements TowerIO
{
    private TalonFX m_leaderFlywheel   = new TalonFX(Constants.CanIds.FlywheelMotorId);
    private TalonFX m_followerFlywheel = new TalonFX(Constants.CanIds.FlywheelFollowerMotorId);

    private double m_setTPS = 0.0;

    public Tower()
    {
        TalonFXConfig.configure(m_leaderFlywheel,
                        85,            // Current limit
                        false,            // Inverted
                        false,           // Brake mode
                        false,           // Continuous wrap
                        0.55,            // P gain
                        0.0,             // I gain
                        0.0,             // D gain
                        0.0,             // S (static friction feedforward)
                        0.13,            // V (velocity feedforward)
                        0.99,            // A (acceleration feedforward)
                        0.0,           // Velocity limit
                        0.0);  // Acceleration limit

        m_followerFlywheel.setControl(new Follower(Constants.CanIds.FlywheelMotorId, MotorAlignmentValue.Opposed));
    }

    // In rotations per second
    public void setSpeed(double speed)
    {
        m_setTPS = speed;

        if (speed <= 0.5)
        {
            m_leaderFlywheel.setControl(new VoltageOut(0.0));
        }
        else
        {
            m_leaderFlywheel.setControl(new VelocityVoltage(speed));
        }
    }

    public double getDesiredFlywheelTPS() { return m_setTPS; }
    public double getFlywheelTPS()        { return m_leaderFlywheel.getVelocity().getValue().in(RotationsPerSecond); }
}
