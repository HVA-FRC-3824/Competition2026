package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;
import frc.robot.lib.TalonFXConfig;

public class Indexer extends SubsystemBase
{
    private final TalonFX m_beltMotor   = new TalonFX(CanIds.BeltsMotorId);
    private final TalonFX m_kickerMotor = new TalonFX(CanIds.KickerMotorId);

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
            120.0, 
            30.0);
        
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
            120.0, 
            50.0);    
    }

    public void setSpeeds(double beltsSpeed, double kickerSpeed) 
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