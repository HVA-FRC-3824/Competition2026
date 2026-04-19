package frc.robot.lib.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;

public class MotorConfig 
{
    private Current m_statorCurrent = Amps.of(0.0);
    private Current m_supplyCurrent = Amps.of(0.0);
    private boolean m_inverted = false;
    private boolean m_brakeMode = false;
    private boolean m_continuousWrap = false;
    private double  m_P = 0.0; // MUST SET THIS
    private double  m_I = 0.0;
    private double  m_D = 0.0;
    private double  m_S = 0.0;
    private double  m_V = 0.0;
    private double  m_A = 0.0;
    private AngularVelocity     m_velocityLimit     = RotationsPerSecond.of(120.0); // 120 is generally the max
    private AngularAcceleration m_accelerationLimit = RotationsPerSecondPerSecond.of(1200.0); // get to the max in one tenth sec
    private double  m_sensorToMechanismRatio = 1.0;

    public MotorConfig()
    {

    }

    public MotorConfig(
        TalonFX motor,
        Current maxAmperage,
        Current supplyCurrent,
        boolean inverted,
        boolean brakeMode,
        boolean continuousWrap,
        double P, double I, double D,
        double S, double V, double A,
        AngularVelocity     velocityLimit,
        AngularAcceleration accelerationLimit,
        double sensorToMechanismRatio)
    {
        m_statorCurrent = maxAmperage;
        m_supplyCurrent = supplyCurrent;
        m_inverted = inverted;
        m_brakeMode = brakeMode;
        m_continuousWrap = continuousWrap;
        m_P = P;
        m_I = I;
        m_D = D;
        m_S = S;
        m_V = V;
        m_A = A;
        m_velocityLimit = velocityLimit;
        m_accelerationLimit = accelerationLimit;
        m_sensorToMechanismRatio = sensorToMechanismRatio;
    }

    public MotorConfig withStatorLimit(Current maxAmperage)       
    { 
        m_statorCurrent = Amps.of(MathUtil.clamp(maxAmperage.in(Amps), 0.0, Constants.MotorConfigs.MaxCurrent.in(Amps))); return this; 
    }

    public MotorConfig withSupplyLimit(Current maxAmperage)       
    { 
        m_statorCurrent = Amps.of(MathUtil.clamp(maxAmperage.in(Amps), 0.0, Constants.MotorConfigs.MaxCurrent.in(Amps))); return this; 
    }

    public MotorConfig withInverted(boolean inverted)             
    { 
        m_inverted = inverted; return this; 
    }

    public MotorConfig withBrakeMode(boolean brakeMode)           
    { 
        m_brakeMode = brakeMode; return this; 
    }

    public MotorConfig withContinuousWrap(boolean continuousWrap) 
    { 
        m_continuousWrap = continuousWrap; return this; 
    }

    public MotorConfig withP(double P)                            
    { 
        m_P = P; return this; 
    }

    public MotorConfig withI(double I)                            
    { 
        m_I = I; return this; 
    }

    public MotorConfig withD(double D)                            
    { 
        m_D = D; return this; 
    }

    public MotorConfig withS(double S)                            
    { 
        m_S = S; return this; 
    }

    public MotorConfig withV(double V)                            
    { 
        m_V = V; return this; 
    }

    public MotorConfig withA(double A)                            
    { 
        m_A = A; return this; 
    }

    public MotorConfig withVelocityLimit(AngularVelocity velocityLimit)             
    { 
        m_velocityLimit = velocityLimit; return this; 
    }

    public MotorConfig withAccelerationLimit(AngularAcceleration accelerationLimit) 
    { 
        m_accelerationLimit = accelerationLimit; return this; 
    }

    public MotorConfig withSensorToMechanismRatio(double sensorToMechanismRatio)    
    { 
        m_sensorToMechanismRatio = sensorToMechanismRatio; return this; 
    }

    public void Apply(TalonFX m_motor)
    {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimit = m_supplyCurrent.in(Amps);
        config.CurrentLimits.SupplyCurrentLimitEnable = m_supplyCurrent.in(Amps) != 0.0;
        config.CurrentLimits.StatorCurrentLimit = m_statorCurrent.in(Amps);
        config.CurrentLimits.StatorCurrentLimitEnable = m_statorCurrent.in(Amps) != 0.0;

        config.MotorOutput.Inverted = m_inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = m_brakeMode
            ? NeutralModeValue.Brake
            : NeutralModeValue.Coast;
        
        config.Feedback.SensorToMechanismRatio = m_sensorToMechanismRatio;

        config.ClosedLoopGeneral.ContinuousWrap = m_continuousWrap;

        config.Slot0.kP = m_P;
        config.Slot0.kI = m_I;
        config.Slot0.kD = m_D;
        config.Slot0.kS = m_S;
        config.Slot0.kV = m_V;
        config.Slot0.kA = m_A;

        config.MotionMagic.MotionMagicCruiseVelocity = m_velocityLimit.in(RotationsPerSecond);
        config.MotionMagic.MotionMagicAcceleration   = m_accelerationLimit.in(RotationsPerSecondPerSecond);

        m_motor.getConfigurator().apply(config);
    }
}
