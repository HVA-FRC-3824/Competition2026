package frc3824;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants 
{
    
    public static final class VisionConstants
    {
        public static final String kCameraName1 = "LeftCam";
        public static final String kCameraName2 = "RightCam";
        
        public static final Transform3d RobotToCam1 = new Transform3d(new Translation3d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)), new Rotation3d());
        public static final Transform3d CamToRobot1 = RobotToCam1.inverse();

        // some of these probably need to be flipped
        public static final Transform3d kRobotToCam2 = new Transform3d(new Translation3d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)), new Rotation3d());
        public static final Transform3d kCamToRobot2 = kRobotToCam2.inverse();

        public static AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.2, 0.2, 0.5);
        public static final Matrix<N3, N1> kMultiTagStdDevs  = VecBuilder.fill(0.05, 0.05, 0.1);
    }

    public static final class IntakeConstants
    {
        public static final double IntakeStowedDegrees    = 0.0;
        public static final double IntakeDeployedDegrees  = 90;
        public static final double IntakeStartingDegrees  = -21; // This is about the initial offset of the intake at the start

        public static final double IntakeDriveTurnsPerSec = 60;

        public static final double IntakePositionGearReduction = 25.0;
        public static final double IntakeRollerGearReduction   = 10.0;
    }

    public static final class SpindexerConstants
    {
        public static final double SpinnerWheelTurnsPerSec = 130;
        public static final double KickerWheelTurnsPerSec  = 120;
    }

    public static final class ChassisConstants
    {
        // NOTE: The absolute encoder range is 0.5 to -0.5
        // These are the absolute encoder values that correspond to the wheels facing "forward"
        public static final double FrontLeftForwardDegrees  = -0.023438 * 360.0;
        public static final double FrontRightForwardDegrees = -0.362793 * 360.0;
        public static final double BackLeftForwardDegrees   = -0.361328 * 360.0;
        public static final double BackRightForwardDegrees  = -0.276855 * 360.0;

        public static final double MaximumSpeedMetersPerSec = Units.feetToMeters(10.0);
        public static final double MaximumAngularVelocity   = 1 * Math.PI;

        public static final double WheelBaseMeters  = Units.inchesToMeters(30.0);
        public static final double TrackWidthMeters = Units.inchesToMeters(24.0);

        
        public static final double DriveMotorReduction  = 6.75;
        public static final double WheelDiameter        = 0.098022; // meters
        public static final double WheelCircumference   = WheelDiameter * Math.PI;
        public static final double DriveMotorConversion = WheelCircumference / DriveMotorReduction;  // Meters per motor turn

        public static final SwerveModuleState[] xStates = {
            new SwerveModuleState(0.0, new Rotation2d(315.0)),  // FL
            new SwerveModuleState(0.0, new Rotation2d( 45.0)),  // FR
            new SwerveModuleState(0.0, new Rotation2d( 45.0)),  // BL
            new SwerveModuleState(0.0, new Rotation2d(315.0))  // BR
        };

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d( Constants.ChassisConstants.WheelBaseMeters / 2,  Constants.ChassisConstants.TrackWidthMeters / 2), // Front Left
            new Translation2d( Constants.ChassisConstants.WheelBaseMeters / 2, -Constants.ChassisConstants.TrackWidthMeters / 2), // Front Right
            new Translation2d(-Constants.ChassisConstants.WheelBaseMeters / 2,  Constants.ChassisConstants.TrackWidthMeters / 2), // Back Left
            new Translation2d(-Constants.ChassisConstants.WheelBaseMeters / 2, -Constants.ChassisConstants.TrackWidthMeters / 2)  // Back Right
        );

        public static final PathConstraints constraints = new PathConstraints(Constants.ChassisConstants.MaximumSpeedMetersPerSec, 
                                                                              Constants.ChassisConstants.MaximumSpeedMetersPerSec, 
                                                                              Constants.ChassisConstants.MaximumAngularVelocity, 
                                                                              Constants.ChassisConstants.MaximumAngularVelocity);
    }

    public static final class ConstantsCanIds
    {
        public static final int FrontLeftDriveId        = 01; // Kraken X60
        public static final int FrontLeftTurnId         = 02; // Kraken X44
        public static final int FrontLeftEncoderId      = 03; // CANCoder

        public static final int FrontRightDriveId       = 11; // Kraken X60
        public static final int FrontRightTurnId        = 12; // Kraken X44
        public static final int FrontRightEncoderId     = 13; // CANCoder

        public static final int BackLeftDriveId         = 21; // Kraken X60
        public static final int BackLeftTurnId          = 22; // Kraken X44
        public static final int BackLeftEncoderId       = 23; // CANCoder
        
        public static final int BackRightDriveId        = 31; // Kraken X60
        public static final int BackRightTurnId         = 32; // Kraken X44
        public static final int BackRightEncoderId      = 33; // CANCoder

        public static final int IntakePositionMotorId   = 40; // Kraken X60
        public static final int FuelIntakeMotorId       = 41; // Kraken X60

        public static final int SpinnerMotorId          = 50; // Kraken X60
        public static final int KickerMotorId           = 51; // Kraken X44
        public static final int KickerFollowerMotorId   = 52; // Kraken X44
        public static final int TurretMotorId           = 53; // Kraken X44
        public static final int FlywheelMotorId         = 54; // Kraken X60
        public static final int FlywheelFollowerMotorId = 55; // Kraken X60

        public static final int ClimbMotorId            = 60; // Kraken X60
    }

    public static final class ConstantsPwmPorts
    {
        // PWM Ports
        public static final int ActuatorPort     = 1;

        public static final int LedUnderGlowPort = 9;
        public static final int LedTurretPort    = 7;
    }

    public static final class ConstantsUsbPort
    {
        
        // Drive Input Configurations
        public static final int DrivePort    = 0;
        public static final int OperatorPort = 1;
    }
}
