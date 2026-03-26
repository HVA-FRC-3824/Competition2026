package frc3824.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3824.Constants;
import frc3824.lib.TalonFXConfig;

public class Tower extends SubsystemBase
{
    private TalonFX m_leaderFlywheel   = new TalonFX(Constants.CanIds.FlywheelMotorId);
    private TalonFX m_followerFlywheel = new TalonFX(Constants.CanIds.FlywheelFollowerMotorId);

    public Tower()
    {
        TalonFXConfig.configure(m_leaderFlywheel,
                        40,            // Current limit
                        true,            // Inverted
                        false,           // Brake mode
                        false,           // Continuous wrap
                        0.32,            // P gain
                        0.0,             // I gain
                        0.0,             // D gain
                        0.0,             // S (static friction feedforward)
                        0.13,            // V (velocity feedforward)
                        1.61,            // A (acceleration feedforward)
                        0.0,           // Velocity limit
                        0.0);  // Acceleration limit

        m_followerFlywheel.setControl(new Follower(Constants.CanIds.FlywheelMotorId, MotorAlignmentValue.Opposed));
    }

    // In rotations per second
    public void setSpeed(double speed)
    {
        if (Math.abs(speed) <= 0.10)
        {
            m_leaderFlywheel.setControl(new VoltageOut(0.0));
        } 
        else
        {
           m_leaderFlywheel.setControl(new VelocityVoltage(speed));
        }
    }

    public boolean isSpunUp()
    {
        return m_leaderFlywheel.getVelocity().isNear(
            m_leaderFlywheel.getClosedLoopReference().getValue(), 
            Constants.Tower.SpunUpTolerance);
    }
}
