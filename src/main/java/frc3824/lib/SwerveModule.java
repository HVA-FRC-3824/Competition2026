package frc3824.lib;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc3824.Constants.Chassis;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final CANcoder angleAbsoluteEncoder;

    public SwerveModule(int driveMotorCanId, int angleMotorCanId, int angleEncoderCanId) {

        driveMotor = new TalonFX(driveMotorCanId);
        angleMotor = new TalonFX(angleMotorCanId);
        angleAbsoluteEncoder = new CANcoder(angleEncoderCanId);

        // Reset encoders
        driveMotor.setPosition(0);
        angleMotor.setPosition(0);

        // Configure motors (you’ll need your own config helper or inline config)
        TalonFXConfig.configure(
            driveMotor,    // Drive motor configuration
            85,              // Maximum Amperage
            false,           // Inverted
            true,            // Brake mode enabled
            false,           // Continuous wrap
            0.03,            // P gain
            1.5,             // I gain
            0.0,             // D gain
            0.0,             // S gain
            0.12877,             // V gain
            0.0,             // A gain
            0.0,           // Velocity limit
            0.0);  // Acceleration limit

        TalonFXConfig.configure(
            angleMotor,    // Angle motor configuration
            20.0,            // Maximum Amperage
            true,            // Inverted
            true,            // Brake mode enabled
            true,            // Continuous wrap
            10.0,            // P gain   TODO: Try a higher gain when the robot is on the ground
            0.0,             // I gain
            0.2,             // D gain
            0.0,             // V gain
            0.0,             // A gain
            0.0,             // S gain
            0.0,           // Velocity limit
            0.0,   // Acceleration limit
            150.0 / 7.0);    // Sensor to mechanism ratio
    }

    public void setDesiredState(SwerveModuleState desiredState, String description) {

        // Optimize state
        desiredState.optimize(getPosition().angle);

        // Set angle (degrees → rotations)
        double angleRotations = desiredState.angle.getDegrees() / 360.0;
        angleMotor.setControl(new PositionDutyCycle(angleRotations));

        // Set velocity
        double velocity = desiredState.speedMetersPerSecond
                / Chassis.DriveMotorConversion;

        driveMotor.setControl(new VelocityVoltage(velocity));
    }

    public SwerveModuleState getState() {

        double velocity = driveMotor.getVelocity().getValue().in(DegreesPerSecond)
                * Chassis.DriveMotorConversion;

        double angleDeg = angleMotor.getPosition().getValue().in(Degrees);

        return new SwerveModuleState(
                velocity,
                Rotation2d.fromDegrees(angleDeg)
        );
    }

    public SwerveModulePosition getPosition() {

        double distance = driveMotor.getPosition().getValue().in(Rotations)
                * Chassis.DriveMotorConversion;

        double angleDeg = angleMotor.getPosition().getValue().in(Degrees);

        return new SwerveModulePosition(
                distance,
                Rotation2d.fromDegrees(angleDeg)
        );
    }

    public void resetEncoders() {
        driveMotor.setPosition(0);
        angleMotor.setPosition(0);
    }

    public void setWheelAngleToForward(double forwardAngleDeg) {

        driveMotor.setPosition(0);
        angleMotor.setPosition(0);

        double moveDegrees = -1 * (forwardAngleDeg - getAbsoluteEncoderAngle());

        // Wrap between -180 and 180
        while (moveDegrees > 180) moveDegrees -= 360;
        while (moveDegrees < -180) moveDegrees += 360;

        double turns = moveDegrees / 360.0;

        angleMotor.setPosition(turns);
        angleMotor.setControl(new PositionDutyCycle(0));
    }

    private double getAbsoluteEncoderAngle() {
        return angleAbsoluteEncoder.getAbsolutePosition().getValue().in(Degrees);
    }

}