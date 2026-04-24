package frc.robot.subsystems.swerveModule;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.Constants.Chassis;
import frc.robot.lib.motor.talonFX.SimpleTalon;

public class SwerveModuleTalonFx extends SwerveModule
{
  private final SimpleTalon m_drivingMotor;

  private final SimpleTalon m_angleMotor;

  private final CANcoder angleAbsoluteEncoder;

  private final int m_num;

  public SwerveModuleTalonFx(int moduleNum, int driveMotorCanId, int angleMotorCanId, int angleEncoderCanId) {
    
    m_inputs = new Inputs();
    m_outputs = new Outputs();

    m_drivingMotor     = new SimpleTalon(driveMotorCanId, Constants.Chassis.DriveConfig);
    m_angleMotor     = new SimpleTalon(angleMotorCanId, Constants.Chassis.TurnConfig);
    angleAbsoluteEncoder = new CANcoder(angleEncoderCanId);

    CANcoderConfiguration cancoderConfiguration = new CANcoderConfiguration();
    MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
    magnetSensorConfigs.withMagnetOffset(0.0).withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

    angleAbsoluteEncoder.getConfigurator().apply(cancoderConfiguration.withMagnetSensor(magnetSensorConfigs));

    m_num = moduleNum;

    // Reset encoders
    m_drivingMotor.resetEncoder(Rotations.of(0.0));
    m_angleMotor.resetEncoder(Rotations.of(0.0));
  }

  @Override
  protected int getNum() {
    return m_num;
  }

  @Override
  protected void setPosition(Angle angle) {
    m_angleMotor.setPosition(angle);
  }

  @Override
  protected void setVelocity(AngularVelocity velocity) {
    m_drivingMotor.setVelocity(velocity);
  }

  @Override
  protected SwerveModuleState getState() {
    return new SwerveModuleState(
        m_drivingMotor.getVelocity().in(RotationsPerSecond) * Constants.Chassis.DriveMotorConversion,
        new Rotation2d(m_angleMotor.getPos())
    );
  }

  @Override
  protected SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_drivingMotor.getPos().in(Rotations) * Chassis.DriveMotorConversion,
        new Rotation2d(m_angleMotor.getPos())
    );
  }
  
  @Override
  public void resetEncoders() {
    m_drivingMotor.resetEncoder(Rotations.of(0.0));
  }

  @Override
  public void setWheelAngleToForward(Angle forwardAngleDeg) {
    m_angleMotor.resetEncoder(Rotations.of(0.0));

    // Set turn motor relative to the absolute encoder
    Angle moveDegrees = angleAbsoluteEncoder.getAbsolutePosition().getValue().minus(forwardAngleDeg);
    m_angleMotor.resetEncoder(moveDegrees);

    // Snap to 0
    m_angleMotor.setPosition(Rotations.of(0.0));
  }

}