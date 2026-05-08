package frc.robot.lib.motor.io;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.lib.motor.MotorConfig;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volt;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkMaxIO implements MotorIO {
    
  // Chosen by us, this is kinda arbitrary, but 360 works fine
  // Because REV is stupid and uses integer resolution, I can't just use turns and have to do this
  // I could do like 4000.0 but that seems excessive
  private static double EncoderResolution = 360.0;

  private SparkMax        m_motor;
  private SparkMaxConfig  m_motorConfig;
  private RelativeEncoder m_encoder;
  private AbsoluteEncoder m_absEncoder;

  private PIDController m_pidController;
  
  public SparkMaxIO(int id) {
    
    m_motor       = new SparkMax(id, MotorType.kBrushless);
    m_motorConfig = new SparkMaxConfig();
    m_encoder     = m_motor.getEncoder();
    m_absEncoder  = m_motor.getAbsoluteEncoder();

    m_pidController = new PIDController(0.0001, 0.0 ,0.0);

    Logger.runEveryN(1, () -> Logger.recordOutput("MotorErr/SparkMax " + m_motor.getDeviceId(), m_motor.getLastError().toString()));
  }

  public SparkMaxIO(int id, MotorConfig config) {

    this(id);

    config(config);
  }

  public void config(MotorConfig config) {

    m_pidController.setPID(config.getP(), config.getI(), config.getD());

    // Taken from 3140 thx!
    m_pidController.setTolerance(5.0 ,1.0);
    
    if (config.isContinuousWrap()) m_pidController.enableContinuousInput(0.0, 360.0);

    m_motorConfig
      .inverted(config.isInverted())
      .idleMode(config.isBrakeMode() ? IdleMode.kBrake : IdleMode.kCoast);
    
    // m_motorConfig.smartCurrentLimit( 
    //   (int) config.statorCurrent().in(Amps),
    //   (int) config.supplyCurrent().in(Amps));

    m_motorConfig.encoder
      .positionConversionFactor((EncoderResolution / config.getSensorToMechanismRatio()))
      .velocityConversionFactor((EncoderResolution / config.getSensorToMechanismRatio()) / 60.0); // Div by 60 because its in RPM by default
    
    m_motorConfig.absoluteEncoder
      .positionConversionFactor((EncoderResolution / config.getSensorToMechanismRatio()))
      .velocityConversionFactor((EncoderResolution / config.getSensorToMechanismRatio()) / 60.0); // Div by 60 because its in RPM by default

    // Write the configuration to the motor controller
    m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);  
  }

  public void follow(int id, boolean inverted) {

    m_motorConfig.follow(id, inverted);

    m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void brake() {

    m_motor.set(0.0);
  }

  public void setPosition(Angle angle) {

    m_pidController.setSetpoint(angle.in(Degrees));
    m_motor.set(m_pidController.calculate(getPos().in(Degrees))); // could be negative
  }

  public void setVelocity(AngularVelocity angleVel) {
    
    m_motor.set(m_pidController.calculate(getVelocity().in(DegreesPerSecond), angleVel.in(DegreesPerSecond)));
  }

  public void resetEncoder(Angle angle) {

    m_encoder.setPosition(angle.in(Degrees));
  }

  public Voltage getAppliedVoltage() {

    return Voltage.ofRelativeUnits(m_motor.getAppliedOutput() * m_motor.getBusVoltage(), Volt);
  } 

  public Voltage getSupplyVoltage() {

    return Voltage.ofRelativeUnits(m_motor.getBusVoltage(), Volt);
  }

  public Angle getPos() {

    return Rotations.of(m_encoder.getPosition() / EncoderResolution);
  }

  public AngularVelocity getVelocity() {

    return RotationsPerSecond.of(m_encoder.getVelocity() / EncoderResolution);
  }

}
