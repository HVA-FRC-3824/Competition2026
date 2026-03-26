package frc3824.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3824.Constants.CanIds;
import frc3824.lib.TalonFXConfig;

public class Indexer extends SubsystemBase
{

    public enum SpindexerState 
    {
        Stopped,
        Spindexing,
        Backwards
    }

    private final TalonFX m_beltMotor           = new TalonFX(CanIds.BeltsMotorId);
    private final TalonFX m_kickerMotor         = new TalonFX(CanIds.KickerMotorId);
    private final TalonFX m_kickerFollowerMotor = new TalonFX(CanIds.KickerFollowerMotorId);

    private SpindexerState m_state = SpindexerState.Stopped;

    public Indexer() {
        TalonFXConfig.configure(
            m_beltMotor,  
            20.0, 
            true,  
            false, 
            false, 
            0.2, 
            0.0, 
            0.0, 
            0.0, 
            0.0, 
            0.0, 
            0.0, 
            0.0);
        
        TalonFXConfig.configure(
            m_kickerMotor,  
            20.0, 
            false,  
            false, 
            false, 
            0.2, 
            0.0, 
            0.0, 
            0.0, 
            0.0, 
            0.0, 
            0.0, 
            0.0);    
    }

    public void SetSpeeds(double beltsSpeed, double kickerSpeed) 
    {
        if (Math.abs(beltsSpeed) <= 0.10 || Math.abs(kickerSpeed) <= 0.10)
        {
            m_beltMotor.setControl(new VoltageOut(0.0));
            m_kickerMotor.setControl(new VoltageOut(0.0));
        } 
        else
        {
            m_beltMotor.setControl(new MotionMagicVoltage(beltsSpeed));
            m_kickerMotor.setControl(new MotionMagicVoltage(kickerSpeed));
        }
    }
}