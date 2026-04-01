package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
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
            60.0, 
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
            120.0, 
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
        m_beltMotor.setControl(new VelocityVoltage(beltsSpeed));
        m_kickerMotor.setControl(new VelocityVoltage(kickerSpeed));
    }
}