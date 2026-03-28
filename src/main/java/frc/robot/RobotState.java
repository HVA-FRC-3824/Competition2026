package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.chassis.Chassis;

public class RobotState 
{
    public enum TowerState
    {
        Idle,
        Low,
        Middle,
        Long,
        Auto,
        ManualControl
    }

    public enum IndexerState
    {
        Stopped,
        Spindexing,
        Backwards
    }

    public enum LedState
    {
        Off,
        SolidGreen,
        SolidRed,
        HvaColors,
        Strobe,
        Rainbow,
        MatchMode
    }

    public enum IntakePosState
    {
        Stowed,
        Deployed,
        StartingPos
    }

    public enum IntakeRollerState
    {
        Intaking,
        Off,
        Backwards
    }

    public enum ChassisState
    {
        Driving,
        AimHub
    }

    private static TowerState        m_towerState = TowerState.Idle;
    private static IndexerState      m_indexerState = IndexerState.Stopped;
    private static LedState          m_ledState = LedState.MatchMode;
    private static IntakePosState    m_intakePosState = IntakePosState.StartingPos;
    private static IntakeRollerState m_intakeRollerState = IntakeRollerState.Off;
    private static double            m_manualFlywheelSpeed = 55.0;
    private static ChassisState      m_chassisState = ChassisState.Driving;
    private static ChassisSpeeds     m_speeds = new ChassisSpeeds();

    private static Tower       m_tower;
    private static Indexer     m_indexer;
    private static Intake      m_intake;
    private static Chassis     m_chassis;

    private static final PIDController m_aimController = new PIDController(0.5, 0.0, 0.0);

    // Indexer
    public final Runnable indexingCommand          = () -> { m_indexerState = IndexerState.Spindexing; };
    public final Runnable notIndexingCommand       = () -> { m_indexerState = IndexerState.Stopped; };
    public final Runnable backwardsIndexingCommand = () -> { m_indexerState = IndexerState.Backwards; };

    // Intake
    public final Runnable deployIntakeCommand  = () -> { m_intakePosState = IntakePosState.Deployed; };
    public final Runnable retractIntakeCommand = () -> { m_intakePosState = IntakePosState.Stowed; };

    public final Runnable startIntakeCommand   = () -> { m_intakeRollerState = IntakeRollerState.Intaking; };
    public final Runnable stopIntakeCommand    = () -> { m_intakeRollerState = IntakeRollerState.Off; };
    public final Runnable spitOutIntakeCommand = () -> { m_intakeRollerState = IntakeRollerState.Backwards; };

    // Tower
    public final Runnable spinDownTowerCommand  = () -> { m_towerState = TowerState.Idle; };
    public final Runnable lowShootTowerCommand  = () -> { m_towerState = TowerState.Low; };
    public final Runnable midShootTowerCommand  = () -> { m_towerState = TowerState.Middle; };
    public final Runnable longShootTowerCommand = () -> { m_towerState = TowerState.Long; };
    public final Runnable autoTowerCommand      = () -> { m_towerState = TowerState.Auto; };
    public final Runnable manualTowerCommand    = () -> { m_towerState = TowerState.ManualControl; };

    public final Runnable increaseManualTowerCommand = () -> { m_manualFlywheelSpeed += 1.0; };
    public final Runnable decreaseManualTowerCommand = () -> { m_manualFlywheelSpeed -= 1.0; };
    public final Runnable resetManualTowerCommand    = () -> { m_manualFlywheelSpeed  = 30.0; };

    // Chassis
    public final Runnable autoAimCommand   = () -> { m_chassisState = ChassisState.AimHub; };
    public final Runnable driveModeCommand = () -> { m_chassisState = ChassisState.Driving; };

    public final Runnable xModeCommand    = () -> { m_chassis.toggleXMode(); };
    public final Runnable xModeOnCommand  = () -> { if (!m_chassis.getIsXMode()) m_chassis.toggleXMode(); };
    public final Runnable xModeOffCommand = () -> { if (m_chassis.getIsXMode()) m_chassis.toggleXMode(); };
        
    public final Runnable fieldRelativeCommand    = () -> { m_chassis.toggleXMode(); };
    public final Runnable fieldRelativeOnCommand  = () -> { if (!m_chassis.getIsFieldRelative()) m_chassis.toggleFieldCentric(); };
    public final Runnable fieldRelativeOffCommand = () -> { if (m_chassis.getIsFieldRelative()) m_chassis.toggleFieldCentric(); };

    public final Runnable chassisTrackAndShootHub = () -> {
        Transform2d relativeDistance = GetHubPose().minus(getPose());

        Rotation2d angleToHubFromPos = new Rotation2d(Math.atan2(relativeDistance.getY(), relativeDistance.getX()));

        m_speeds.omegaRadiansPerSecond = m_aimController.calculate(getHeading().getRadians(), angleToHubFromPos.getRadians());

        Logger.recordOutput("Testing/Chassis To Hub Speed", Units.radiansToDegrees(m_speeds.omegaRadiansPerSecond));

        if (m_aimController.atSetpoint())
            xModeOnCommand.run();
        else
            xModeOffCommand.run();

        m_chassis.drive(m_speeds);
    };

    class PathplannerSubsystem extends SubsystemBase
    {
        public PathplannerSubsystem()
        {

        }
    }

    public RobotState()
    {
        m_tower   = new Tower();
        m_indexer = new Indexer();
        m_intake  = new Intake();
        m_chassis = new Chassis();

        // Config PID to be tolerant within 5 degrees
        m_aimController.setTolerance(Units.degreesToRadians(5.0));

        PathplannerSubsystem pathplannerSubsystem = new PathplannerSubsystem();        

        // This should be in robot container
        NamedCommands.registerCommand("index", pathplannerSubsystem.runOnce(indexingCommand));
        NamedCommands.registerCommand("stopIndexing", pathplannerSubsystem.runOnce(notIndexingCommand));

        NamedCommands.registerCommand("stopShoot", pathplannerSubsystem.runOnce(spinDownTowerCommand));
        NamedCommands.registerCommand("lowShoot",  pathplannerSubsystem.runOnce(lowShootTowerCommand));
        NamedCommands.registerCommand("midShoot",  pathplannerSubsystem.runOnce(midShootTowerCommand));
        NamedCommands.registerCommand("longShoot", pathplannerSubsystem.runOnce(longShootTowerCommand));

        NamedCommands.registerCommand("xModeOn",  pathplannerSubsystem.runOnce(xModeOnCommand));
        NamedCommands.registerCommand("xModeOff", pathplannerSubsystem.runOnce(xModeOffCommand));

        NamedCommands.registerCommand("aimMode",   pathplannerSubsystem.runOnce(autoAimCommand));
        NamedCommands.registerCommand("driveMode", pathplannerSubsystem.runOnce(driveModeCommand));
    }

    public void Logging()
    {
        // What it is

        Logger.recordOutput("Measured/Chassis/Speeds",   m_chassis.getSpeeds());
        Logger.recordOutput("Measured/Chassis/States",   m_chassis.getModuleStates());
        Logger.recordOutput("Measured/Chassis/Is Aimed", m_aimController.atSetpoint());

        Logger.recordOutput("Measured/Pose",         m_chassis.getPose());
        Logger.recordOutput("Measured/Heading",      m_chassis.getHeading());
        Logger.recordOutput("Measured/Cam 1 Result", Vision.getResult1());
        Logger.recordOutput("Measured/Cam 2 Result", Vision.getResult2());

        Logger.recordOutput("Measured/Tower/Is Spun Up", m_tower.isSpunUp());
        Logger.recordOutput("Measured/Tower/TPS",        m_tower.getFlywheelTPS());

        // What we want

        Logger.recordOutput("Desired/Chassis/Speeds",    m_speeds);
        Logger.recordOutput("Desired/Chassis/XMode",     m_chassis.getIsXMode());

        Logger.recordOutput("Desired/Tower/TPS",        m_tower.getDesiredFlywheelTPS());
        Logger.recordOutput("Desired/Tower/Manual TPS", m_manualFlywheelSpeed);

        Logger.recordOutput("Desired/Tower/State",   m_towerState.toString());
        Logger.recordOutput("Desired/Chassis/State", m_chassisState.toString());
        Logger.recordOutput("Desired/Indexer/State", m_indexerState.toString());
        Logger.recordOutput("Desired/Intake/State",  m_intakePosState.toString() + " " + m_intakeRollerState.toString());
        Logger.recordOutput("Desired/Led/State", "No Tyler, there's no LEDs");
        
    }

    public void Periodic()
    {
        Logging();

        m_manualFlywheelSpeed = SmartDashboard.getNumber("Manual flywheel speed", m_manualFlywheelSpeed);
        // SmartDashboard.putData(manualTowerCommand);

        switch (m_towerState)
        {
            case Idle:
                m_tower.setSpeed(0.0);
                break;
            case Low:
                m_tower.setSpeed(Constants.Tower.CloseSpeed);
                break;
            case Middle:
                m_tower.setSpeed(Constants.Tower.MiddleSpeed);
                break;
            case Long:
                m_tower.setSpeed(Constants.Tower.LongSpeed);
                break;
            case Auto:
                double dist = GetHubDistMeters();
                // TODO:
                m_tower.setSpeed(0.0);
                break;
            case ManualControl:
                m_tower.setSpeed(m_manualFlywheelSpeed);
                break;
        }

        switch (m_indexerState)
        {
            case Stopped:
                m_indexer.setSpeeds(0.0, 0.0);
                break;
            case Spindexing:
                m_indexer.setSpeeds(Constants.Indexer.BeltTurnsPerSec, Constants.Indexer.KickerWheelTurnsPerSec);
                break;
            case Backwards:
                m_indexer.setSpeeds(-Constants.Indexer.BeltTurnsPerSec, -Constants.Indexer.KickerWheelTurnsPerSec);
                break;
        }
    
        switch (m_intakePosState)
        {
            case Stowed:
                m_intake.setPos(Constants.Intake.IntakeStowedTurns);
                break;
            case Deployed:
                m_intake.setPos(Constants.Intake.IntakeDeployedTurns);
                break;
            case StartingPos:
                Logger.recordOutput("Why ", "are you going to the starting pos");
                break;
        }

        switch (m_intakeRollerState)
        {
            case Intaking:
                m_intake.setRollers(Constants.Intake.IntakeDriveTurnsPerSec);
                break;
            case Off:
                m_intake.setRollers(0.0);
                break;
            case Backwards:
                m_intake.setRollers(-Constants.Intake.IntakeDriveTurnsPerSec);
                break;
        }

        switch (m_chassisState)
        {
            case Driving:
                m_chassis.drive(m_speeds);
                break;
            case AimHub:
                chassisTrackAndShootHub.run();
                break;
        }
    }

    public double GetHubDistMeters()
    {
        return Math.sqrt(Math.pow(GetHubPose().getMeasureX().in(Meters) - GetHubPose().getMeasureX().in(Meters), 2) + 
                         Math.pow(GetHubPose().getMeasureY().in(Meters) - GetHubPose().getMeasureY().in(Meters), 2));
    }

    public Pose2d GetHubPose()
    {
        return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red ? 
                        Constants.Field.RedHub.toPose2d() : Constants.Field.BlueHub.toPose2d();
    }

    public Pose2d getPose()
    {
        return m_chassis.getPose();
    }

    public Rotation2d getHeading()
    {
        return m_chassis.getHeading();
    }

    public Boolean GetIsReady()
    {
        // If the flywheel is spun up and we're aiming at the target, shoot
        return m_tower.isSpunUp() && 
               m_aimController.atSetpoint() && 
               m_chassisState == ChassisState.AimHub;
    }

    public void setDrive(double leftY, double leftX, double rightX)
    {
        m_speeds = new ChassisSpeeds(Math.pow(Math.abs(leftY), Constants.Chassis.TranslateExponentialPower) * leftY  * Constants.Chassis.MaximumSpeedMetersPerSec,
                                     Math.pow(Math.abs(leftX), Constants.Chassis.TranslateExponentialPower) * leftX  * Constants.Chassis.MaximumSpeedMetersPerSec, 
                                     rightX * Constants.Chassis.MaximumAngularVelocity);
    }
}
