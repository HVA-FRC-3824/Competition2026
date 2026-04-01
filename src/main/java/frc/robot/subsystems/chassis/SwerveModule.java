package frc.robot.subsystems.chassis;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.Chassis;
import frc.robot.lib.TalonFXConfig;

public class SwerveModule
{
    private final TalonFX         m_drivingMotor;

    private final TalonFX         m_angleMotor;

    private final CANcoder angleAbsoluteEncoder;

    public SwerveModule(int driveMotorCanId, int angleMotorCanId, int angleEncoderCanId) {

        m_drivingMotor = new TalonFX(driveMotorCanId);
        m_angleMotor = new TalonFX(angleMotorCanId);
        angleAbsoluteEncoder = new CANcoder(angleEncoderCanId);

        CANcoderConfiguration c = new CANcoderConfiguration();
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.withMagnetOffset(0.0).withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

        angleAbsoluteEncoder.getConfigurator().apply(c.withMagnetSensor(magnetSensorConfigs));

        // Reset encoders
        m_drivingMotor.setPosition(0);
        m_angleMotor.setPosition(0);

        // Configure motors (you’ll need your own config helper or inline config)
        TalonFXConfig.configure(
            m_drivingMotor,    // drive motor configuration
            120,              // Maximum Amperage
            false,           // Inverted
            true,            // Brake mode enabled
            false,           // Continuous wrap
            0.03,            // P gain
            1.5,             // I gain
            0.0,             // D gain
            0.0,             // S gain
            0.12877,         // V gain
            0.0,             // A gain
            0.0,           // Velocity limit
            0.0);  // Acceleration limit

        TalonFXConfig.configure(
            m_angleMotor,    // Angle motor configuration
            70.0,            // Maximum Amperage
            true,            // Inverted
            true,            // Brake mode enabled
            true,            // Continuous wrap
            12.0,            // P gain   TODO: Try a higher gain when the robot is on the ground
            0.0,             // I gain
            0.2,             // D gain
            0.0,             // V gain
            0.0,             // A gain
            0.0,             // S gain
            0.0,           // Velocity limit
            0.0,   // Acceleration limit
            150.0 / 7.0);    // Sensor to mechanism ratio
        
        m_drivingMotor.setPosition(0);
    }

    public void setDesiredState(SwerveModuleState desiredState, String description) 
    {
        // Optimize state
        desiredState.optimize(getPosition().angle);

        // Set angle (degrees → rotations)
        double angleRotations = desiredState.angle.getDegrees() / 360.0;
        m_angleMotor.setControl(new PositionDutyCycle(angleRotations));

        // Set velocity
        double velocity = desiredState.speedMetersPerSecond
                / Constants.Chassis.DriveMotorConversion;

        m_drivingMotor.setControl(new VelocityVoltage(velocity));
    }

    public SwerveModuleState getState() {

        double velocity = m_drivingMotor.getVelocity().getValue().in(DegreesPerSecond)
                * Constants.Chassis.DriveMotorConversion;

        double angleDeg = m_angleMotor.getPosition().getValue().in(Degrees);

        return new SwerveModuleState(
                velocity,
                Rotation2d.fromDegrees(angleDeg)
        );
    }

    public SwerveModulePosition getPosition() {

        double distance = m_drivingMotor.getPosition().getValue().in(Rotations)
                * Chassis.DriveMotorConversion;

        double angleDeg = m_angleMotor.getPosition().getValue().in(Degrees);

        return new SwerveModulePosition(
                distance,
                Rotation2d.fromDegrees(angleDeg)
        );
    }
    
    public void resetEncoders() {
        m_drivingMotor.setPosition(0);
    }

    public void setWheelAngleToForward(double forwardAngleDeg) 
    {
        m_drivingMotor.setPosition(0);
        m_angleMotor.setPosition(0);

        double moveDegrees = -1 * (forwardAngleDeg - (getAbsoluteEncoderAngle() / 360));

        m_angleMotor.setPosition(moveDegrees);
        m_angleMotor.setControl(new PositionDutyCycle(0));
    }

    private double getAbsoluteEncoderAngle() 
    {
        return angleAbsoluteEncoder.getAbsolutePosition().getValue().in(Degrees);
    }

}