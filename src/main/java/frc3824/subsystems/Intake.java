package frc3824.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3824.Constants;

import static edu.wpi.first.units.Units.*;


public class Intake extends SubsystemBase {

    public enum IntakeState {
        DeployedRollerOn,
        StowedOn,
        Stowed,
        DeployedRollerOff
    }

    private final TalonFX m_fuelIntakeMotor; // ID 41 
    private final TalonFX m_intakePositionMotor; // ID 40
    private IntakeState m_IntakeState = IntakeState.Stowed;

    public Intake() {

        m_fuelIntakeMotor = new TalonFX(0); // ID 41 
        m_intakePositionMotor = new TalonFX(0); // ID 40

        
        configureTalonFX(
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

        configureTalonFX(
            m_intakePositionMotor,
            20.0,
            true,
            false,
            false,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            20.0,
            40.0
        );

        m_fuelIntakeMotor.setPosition(0.0);

        m_intakePositionMotor.setPosition(Constants.IntakeConstants.IntakeStartingDegrees / 360.0);
    }

    private void configureTalonFX(
        TalonFX motor,
        double maxAmperage,
        boolean inverted,
        boolean brakeMode,
        boolean continuousWrap,
        double kP, double kI, double kD,
        double kS, double kV, double kA,
        double velocityLimit,
        double accelerationLimit)
    {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = maxAmperage;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotorOutput.Inverted = inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = brakeMode
            ? NeutralModeValue.Brake
            : NeutralModeValue.Coast;
        
        config.ClosedLoopGeneral.ContinuousWrap = continuousWrap;

        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;

        config.MotionMagic.MotionMagicCruiseVelocity = velocityLimit;
        config.MotionMagic.MotionMagicAcceleration = accelerationLimit;

        motor.getConfigurator().apply(config);
    }

    public void setState(IntakeState newState) {
        if (newState == m_IntakeState) return;

        m_IntakeState = newState;

        System.out.println("Setting intake state to: " + newState.ordinal());

        double position = Constants.IntakeConstants.IntakeStowedDegrees;
        double roller = 0.0;

        switch (newState) {
            case DeployedRollerOn:
                position = Constants.IntakeConstants.IntakeDeployedDegrees;
                roller = Constants.IntakeConstants.IntakeDriveTurnsPerSec;
                break;
            case StowedOn:
                position = Constants.IntakeConstants.IntakeStowedDegrees;
                roller = Constants.IntakeConstants.IntakeDriveTurnsPerSec;
                break;
            case Stowed:
                break;
            case DeployedRollerOff:
                position = Constants.IntakeConstants.IntakeDeployedDegrees;
                break;
        }
        System.out.println("Setting position to " + position + "turns, roller to " + roller + " tps");

        double positionRotations = (position/ 360.0) * Constants.IntakeConstants.IntakePositionGearReduction;
        double rollerRotations = roller * Constants.IntakeConstants.IntakeRollerGearReduction;

        m_intakePositionMotor.setControl(new MotionMagicVoltage(positionRotations));
        m_fuelIntakeMotor.setControl(new MotionMagicVoltage(rollerRotations));
    }
}