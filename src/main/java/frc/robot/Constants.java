package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class Constants 
{
    public static final class Field
    {
        /// *** Field Dimensions *** ///
        public static final double FieldLengthMeters  = Units.inchesToMeters(652.11); // 16.56 meters
        public static final double FieldWidthMeters   = Units.inchesToMeters(317.69); //  8.07 meters

        public static final double AllianceWallToAllianceZoneMeters = Units.inchesToMeters(182.11);

        public static final double HubHeightMeters = Units.inchesToMeters(72.0);

        /// *** Field Locations *** ///

        public static final Pose3d BlueHub = new Pose3d(AllianceWallToAllianceZoneMeters,                     FieldWidthMeters / 2, HubHeightMeters, new Rotation3d());
        public static final Pose3d RedHub  = new Pose3d(FieldLengthMeters - AllianceWallToAllianceZoneMeters, FieldWidthMeters / 2, HubHeightMeters, new Rotation3d());

        // For passing we want to aim towards the inside of our alliance zone or towards the neutral zone whichever is closer
        // Either way we want the balls to be going as close to our alliance zone as possible, so aim for that
        // - "Aim for the stars and maybe you'll reach the neutral zone" or something like that...

        public static final Pose2d BlueAllianceZoneClose = new Pose2d(AllianceWallToAllianceZoneMeters, FieldWidthMeters / 4, new Rotation2d());
        public static final Pose2d BlueAllianceZoneFar   = new Pose2d(AllianceWallToAllianceZoneMeters, FieldWidthMeters - (FieldWidthMeters / 4), new Rotation2d());

        public static final Pose2d RedAllianceZoneClose = new Pose2d(FieldLengthMeters - AllianceWallToAllianceZoneMeters, FieldWidthMeters / 4, new Rotation2d());
        public static final Pose2d RedAllianceZoneFar   = new Pose2d(FieldLengthMeters - AllianceWallToAllianceZoneMeters, FieldWidthMeters - (FieldWidthMeters / 4), new Rotation2d());
    }
    
    public static final class Vision
    {
        public static final String kCameraName1 = "LeftCam";
        public static final String kCameraName2 = "RightCam";
        
        public static final Transform3d RobotToCam1 = new Transform3d(
            new Translation3d(Units.inchesToMeters(-12.0), Units.inchesToMeters(-6.0), Units.inchesToMeters(12.5)), 
            new Rotation3d(0.0, Units.degreesToRadians(10), Units.degreesToRadians(180)));
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

    public static final class Intake
    {
        public static final double IntakeStowedTurns    = 0.8;
        public static final double IntakeDeployedTurns  = 9.6;

        public static final double IntakeDriveTurnsPerSec = 600;

        public static final double IntakePositionGearReduction = 25.0;
        public static final double IntakeRollerGearReduction   = 10.0;
    }

    public static final class Indexer
    {
        public static final double BeltTurnsPerSec = 1300;
        public static final double KickerWheelTurnsPerSec  = 120;
    }

    public static final class Tower
    {
        public static final double CloseSpeed      = 45.0; // 90 in
        public static final double MiddleSpeed     = 51.0; // 120 in
        public static final double LongSpeed       = 55.0; // 150 in

        public static final double SpunUpTolerance = 2.0;
    }

    public static final class Chassis
    {
        // NOTE: The absolute encoder range is 0.5 to -0.5
        // These are the absolute encoder values that correspond to the wheels facing "forward"
        public static final double FrontLeftForwardDegrees  =  0.3824; // WE ARE SO COOKED
        public static final double FrontRightForwardDegrees =  0.408691;
        public static final double BackRightForwardDegrees  = -0.11377;
        public static final double BackLeftForwardDegrees   = -0.025146;

        public static final double MaximumSpeedMetersPerSec  = Units.feetToMeters(12.0);
        public static final double SlowSpeedMetersPerSec     = Units.feetToMeters(6.0);
        public static       boolean isSlowFlag = false;
        public static final double MaximumAngularVelocity    = 4 * Math.PI;
        public static final double TranslateExponentialPower = 3.0;
        public static final double AngularExponentialPower   = 4.0;

        public static final double WheelBaseMeters  = Units.inchesToMeters(30.0);
        public static final double TrackWidthMeters = Units.inchesToMeters(24.0);

        public static final double DriveMotorReduction  = 6.75;
        public static final double WheelDiameter        = 0.098022; // meters
        public static final double WheelCircumference   = WheelDiameter * Math.PI;
        public static final double DriveMotorConversion = WheelCircumference / DriveMotorReduction;  // Meters per motor turn

        public static final SwerveModuleState[] XishStates = {
            new SwerveModuleState(0.0, new Rotation2d(315.0)),  // FL
            new SwerveModuleState(0.0, new Rotation2d( 225.0)),  // FR
            new SwerveModuleState(0.0, new Rotation2d( 225.0)),  // BL
            new SwerveModuleState(0.0, new Rotation2d(315.0))  // BR
        };

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d( Constants.Chassis.WheelBaseMeters / 2,  Constants.Chassis.TrackWidthMeters / 2), // Front Left
            new Translation2d( Constants.Chassis.WheelBaseMeters / 2, -Constants.Chassis.TrackWidthMeters / 2), // Front Right
            new Translation2d(-Constants.Chassis.WheelBaseMeters / 2,  Constants.Chassis.TrackWidthMeters / 2), // Back Left
            new Translation2d(-Constants.Chassis.WheelBaseMeters / 2, -Constants.Chassis.TrackWidthMeters / 2)  // Back Right
        );

        public static final PathConstraints constraints = new PathConstraints(Constants.Chassis.MaximumSpeedMetersPerSec, 
                                                                              Constants.Chassis.MaximumSpeedMetersPerSec, 
                                                                              Constants.Chassis.MaximumAngularVelocity, 
                                                                              Constants.Chassis.MaximumAngularVelocity);
    }

    public static final class Controller
    {
                // Buttons
        public static final int A                   =   1;
        public static final int B                   =   2;
        public static final int X                   =   3;
        public static final int Y                   =   4;
        public static final int LeftBumper          =   5;
        public static final int RightBumper         =   6;
        public static final int Back                =   7;
        public static final int Start               =   8;
        public static final int LeftPaddle          =   9;
        public static final int RightPaddle         =  10;

        public static final int Pov_0               =   0;
        public static final int Pov_45              =  45;
        public static final int Pov_90              =  90;
        public static final int Pov_135             = 135;
        public static final int Pov_180             = 180;
        public static final int Pov_225             = 225;
        public static final int Pov_270             = 270;
        public static final int Pov_315             = 315;
    }

    public static final class CanIds
    {
        public static final int FrontLeftDriveId        = 31; // Kraken X60
        public static final int FrontLeftTurnId         = 32; // Kraken X44
        public static final int FrontLeftEncoderId      = 33; // CANCoder

        public static final int FrontRightDriveId       = 21; // Kraken X60
        public static final int FrontRightTurnId        = 22; // Kraken X44
        public static final int FrontRightEncoderId     = 23; // CANCoder

        public static final int BackLeftDriveId         = 11; // Kraken X60
        public static final int BackLeftTurnId          = 12; // Kraken X44
        public static final int BackLeftEncoderId       = 13; // CANCoder
        
        public static final int BackRightDriveId        = 04; // Kraken X60
        public static final int BackRightTurnId         = 02; // Kraken X44
        public static final int BackRightEncoderId      = 03; // CANCoder

        public static final int PigeonGyroId = 05; // CTR Pigeon 2.0

        public static final int IntakePositionFollowerMotorId = 42; // Kraken X44
        public static final int IntakePositionLeaderMotorId   = 41; // Kraken X44
        public static final int FuelIntakeMotorId             = 40; // Kraken X60

        public static final int BeltsMotorId            = 50; // Kraken X60
        public static final int KickerMotorId           = 51; // Kraken X44

        public static final int FlywheelMotorId         = 52; // Kraken X60
        public static final int FlywheelFollowerMotorId = 53; // Kraken X60

        public static final int PdhId                   = 60; // PDH
    }

    public static final class Pwm
    {
        // PWM Ports
        public static final int ActuatorPort     = 1;

        public static final int LedUnderGlowPort = 9;
        public static final int LedTurretPort    = 7;
    }

    public static final class Usb
    {   
        // drive Input Configurations
        public static final int DrivePort    = 0;
        public static final int OperatorPort = 1;
    }
}
