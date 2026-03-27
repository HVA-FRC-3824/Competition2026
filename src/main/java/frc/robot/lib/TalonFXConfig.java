package frc.robot.lib;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TalonFXConfig {
    public static void configure(
        TalonFX motor,
        double maxAmperage,
        boolean inverted,
        boolean brakeMode,
        boolean continuousWrap,
        double kP,
        double kI, 
        double kD,
        double kS,
        double kV,
        double kA,
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

    public static void configure(
        TalonFX motor,
        double maxAmperage,
        boolean inverted,
        boolean brakeMode,
        boolean continuousWrap,
        double kP, double kI, double kD,
        double kS, double kV, double kA,
        double velocityLimit,
        double accelerationLimit,
        double sensorToMechanismRatio)
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
        
        config.Feedback.SensorToMechanismRatio = sensorToMechanismRatio;

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
}
