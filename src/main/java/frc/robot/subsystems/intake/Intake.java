package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.Constants;
import frc.robot.lib.TalonFXConfig;

public class Intake implements IntakeIO
{
    private final TalonFX m_fuelIntakeMotor             = new TalonFX(Constants.CanIds.FuelIntakeMotorId);
    private final TalonFX m_intakePositionLeaderMotor   = new TalonFX(Constants.CanIds.IntakePositionLeaderMotorId);
    private final TalonFX m_intakePositionFollowerMotor = new TalonFX(Constants.CanIds.IntakePositionFollowerMotorId);

    public Intake() 
    {    
        TalonFXConfig.configure(
            m_fuelIntakeMotor,
            120.0,
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
            50.0,
            true,
            true,
            false,
            0.8,
            0.0,
            0.0,
            0.8,
            0.0,
            0.0,
            5.0,
            5.0
        );

        TalonFXConfig.configure(
            m_intakePositionFollowerMotor,
            50.0,
            false,
            true,
            false,
            0.8,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            5.0,
            5.0
        );
        

        m_fuelIntakeMotor.setPosition(0.0);

        m_intakePositionFollowerMotor.setControl(
            new Follower(Constants.CanIds.IntakePositionLeaderMotorId, MotorAlignmentValue.Opposed));
    }

    // pos is in turns
    public void setPos(double pos) 
    {
        m_intakePositionLeaderMotor.setControl(new MotionMagicVoltage(pos));
    }

    // turns per second
    public void setRollers(double speed)
    {
        if (speed <= 0.5)
        {
            m_fuelIntakeMotor.setControl(new VoltageOut(0.0));
        }
        else
        {
            m_fuelIntakeMotor.setControl(new VoltageOut(12.0));
        }
    }
}