package frc.robot.subsystems.swerveModule;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.Constants.Chassis;
import frc.robot.lib.motor.io.MotorIO;
import frc.robot.lib.motor.io.SparkMaxIO;
import frc.robot.lib.motor.io.TalonIO;

public class SwerveModuleIOMixed implements SwerveModuleIO
{
  private final MotorIO m_drivingMotor;
  private final MotorIO m_angleMotor;

  private final CANcoder angleAbsoluteEncoder;

  private final int m_num;

  private Angle m_forwardsAngle;

  public SwerveModuleIOMixed(int moduleNum, int driveMotorCanId, int angleMotorCanId, int angleEncoderCanId, Angle forwardsAngle) {
    
    m_drivingMotor = new TalonIO(driveMotorCanId, Constants.Chassis.SummerDriveConfig);
    m_angleMotor   = new SparkMaxIO(angleMotorCanId, Constants.Chassis.SummerTurnConfig);
    angleAbsoluteEncoder = new CANcoder(angleEncoderCanId);

    m_forwardsAngle = forwardsAngle;

    CANcoderConfiguration cancoderConfiguration = new CANcoderConfiguration();
    MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
    magnetSensorConfigs.withMagnetOffset(0.0).withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

    angleAbsoluteEncoder.getConfigurator().apply(cancoderConfiguration.withMagnetSensor(magnetSensorConfigs));

    m_num = moduleNum;

    // Reset encoders
    m_drivingMotor.resetEncoder(Rotations.of(0.0));
    m_angleMotor.resetEncoder(Rotations.of(0.0));

    setWheelAngleToForward();
  }

  @Override
  public int getNum() {

    return m_num;
  }

  @Override
  public void setState(SwerveModuleInputs state) {

    Logger.recordOutput("Testing/help", state.angle());

    m_angleMotor.setPosition(state.angle());
    m_drivingMotor.setVelocity(state.velocity());
  }

  @Override
  public void brake() {

    m_angleMotor.brake();
    m_drivingMotor.brake();
  }

  @Override
  public SwerveModuleState getState() {

    return new SwerveModuleState(
        m_drivingMotor.getVelocity().in(RotationsPerSecond) * Constants.Chassis.DriveMotorConversion,
        new Rotation2d(m_angleMotor.getPos())
    );
  }

  @Override
  public SwerveModulePosition getPosition() {
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
  public void setWheelAngleToForward() {

    Angle moveDegrees = m_forwardsAngle.minus(angleAbsoluteEncoder.getAbsolutePosition().getValue()).times(-1.0);

    m_angleMotor.resetEncoder(moveDegrees);
    m_angleMotor.setPosition(Rotations.of(0.0));
  }

}