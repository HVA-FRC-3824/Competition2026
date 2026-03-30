package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.TalonFXConfig;



public class Intake extends SubsystemBase
{
    private final TalonFX m_fuelIntakeMotor             = new TalonFX(Constants.CanIds.FuelIntakeMotorId);
    private final TalonFX m_intakePositionLeaderMotor   = new TalonFX(Constants.CanIds.IntakePositionLeaderMotorId);
    private final TalonFX m_intakePositionFollowerMotor = new TalonFX(Constants.CanIds.IntakePositionFollowerMotorId);

    public Intake() 
    {    
        TalonFXConfig.configure(
            m_fuelIntakeMotor,
            40.0,
            false,
            true,
            false,
            0.2,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        );

        TalonFXConfig.configure(
            m_intakePositionLeaderMotor,
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
            20.0,
            40.0
        );

        m_fuelIntakeMotor.setPosition(0.0);

        m_intakePositionFollowerMotor.setControl(
            new Follower(Constants.CanIds.IntakePositionLeaderMotorId, MotorAlignmentValue.Opposed));
    }

    // pos is in turns
    public void setPos(double pos) 
    {
        // m_intakePositionLeaderMotor.setControl(new MotionMagicVoltage(pos));
    }

    // turns per second
    public void setRollers(double speed)
    {
        if (Math.abs(speed) <= 0.10)
        {
            m_fuelIntakeMotor.setControl(new VoltageOut(0.0));
        } 
        else
        {
            m_fuelIntakeMotor.setControl(new MotionMagicVoltage(speed));
        }
    }
}