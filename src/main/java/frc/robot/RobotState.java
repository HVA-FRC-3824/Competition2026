package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Meters;

import java.util.Arrays;
import java.util.Optional;

import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.ChassisIO;
import frc.robot.subsystems.chassis.SimChassis;
import frc.robot.subsystems.intake.Intake;

public class RobotState 
{
    public enum TowerState
    {
        Low,
        Middle,
        Long,
        Auto,
        ManualControl
    }

    public enum TowerRunningState
    {
        Off,
        On
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
        Pathplanning,
        Driving,
        AimHub
    }

    public  static TowerState        m_towerState = TowerState.Middle;
    private static TowerRunningState m_towerRunningState = TowerRunningState.Off;
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
    private static ChassisIO   m_chassis;

    private static final PIDController m_aimController = new PIDController(0.6, 0.0, 0.0);

    // Indexer
    public final Runnable indexingCommand          = () -> m_indexerState = IndexerState.Spindexing;
    public final Runnable notIndexingCommand       = () -> m_indexerState = IndexerState.Stopped;
    public final Runnable backwardsIndexingCommand = () -> m_indexerState = IndexerState.Backwards;

    // Intake
    public final Runnable deployIntakeCommand  = () -> m_intakePosState = IntakePosState.Deployed;
    public final Runnable retractIntakeCommand = () -> m_intakePosState = IntakePosState.Stowed;
    public final Runnable jiggleIntakeCommand  = () -> { 
        if (((int)Timer.getMatchTime()) % 2 == 0)
            m_intakePosState = IntakePosState.Stowed;
        else
            m_intakePosState = IntakePosState.Deployed;
    };

    public final Runnable startIntakeCommand   = () -> m_intakeRollerState = IntakeRollerState.Intaking;
    public final Runnable stopIntakeCommand    = () -> m_intakeRollerState = IntakeRollerState.Off;
    public final Runnable spitOutIntakeCommand = () -> m_intakeRollerState = IntakeRollerState.Backwards;

    // Tower
    public final Runnable spinDownTowerCommand = () -> m_towerRunningState = TowerRunningState.Off;
    public final Runnable spinUpTowerCommand   = () -> m_towerRunningState = TowerRunningState.On;

    public final Runnable lowShootTowerCommand  = () -> m_towerState = TowerState.Low;
    public final Runnable midShootTowerCommand  = () -> m_towerState = TowerState.Middle;
    public final Runnable longShootTowerCommand = () -> m_towerState = TowerState.Long;
    public final Runnable autoTowerCommand      = () -> m_towerState = TowerState.Auto;
    public final Runnable manualTowerCommand    = () -> m_towerState = TowerState.ManualControl;

    public final Runnable increaseManualTowerCommand = () -> m_manualFlywheelSpeed += 1.0;
    public final Runnable decreaseManualTowerCommand = () -> m_manualFlywheelSpeed -= 1.0;
    public final Runnable resetManualTowerCommand    = () -> m_manualFlywheelSpeed  = 30.0;

    // Chassis
    public final Runnable autoAimCommand      = () -> m_chassisState = ChassisState.AimHub;
    public final Runnable driveModeCommand    = () -> m_chassisState = ChassisState.Driving;
    public final Runnable pathplanningCommand = () -> m_chassisState = ChassisState.Pathplanning;

    public final Runnable xModeCommand    = () -> m_chassis.toggleXMode();
    public final Runnable xModeOnCommand  = () -> { if (!m_chassis.getIsXMode()) m_chassis.toggleXMode(); };
    public final Runnable xModeOffCommand = () -> { if (m_chassis.getIsXMode()) m_chassis.toggleXMode(); };
        
    public final Runnable chassisTrackAndShootHub = () -> {
        Transform2d relativeDistance = getTargetPose().minus(getPose());

        Rotation2d angleToHubFromPos = new Rotation2d(Math.atan2(relativeDistance.getY(), relativeDistance.getX()));

        m_speeds.omegaRadiansPerSecond = m_aimController.calculate(getHeading().getRadians() * -1, angleToHubFromPos.getRadians());

        if (m_aimController.atSetpoint())
        {
            xModeOnCommand.run();
        }
        else
        {
            xModeOffCommand.run();
        }

        m_chassis.driveFieldRelative(m_speeds);
    };

    class PathplannerSubsystem extends SubsystemBase
    {
        public PathplannerSubsystem() {}
    }

    public RobotState()
    {
        m_tower   = new Tower();
        m_indexer = new Indexer();
        m_intake  = new Intake();
        m_chassis = (RobotBase.isSimulation()) ? new SimChassis() : new Chassis();

        m_tower.setDefaultCommand(m_tower.runOnce(()->{}));
        m_indexer.setDefaultCommand(m_indexer.runOnce(()->{}));
        m_intake.setDefaultCommand(m_intake.runOnce(()->{}));
        m_chassis.setDefaultCommand(m_chassis.runOnce(()->{}));

        // Config PID to be tolerant within 5 degrees
        m_aimController.setTolerance(Units.degreesToRadians(1.0));

        PathplannerSubsystem pathplannerSubsystem = new PathplannerSubsystem();        

        // This should be in robot container
        NamedCommands.registerCommand("index", pathplannerSubsystem.runOnce(indexingCommand));
        NamedCommands.registerCommand("stopIndexing", pathplannerSubsystem.runOnce(notIndexingCommand));

        NamedCommands.registerCommand("autoShoot", pathplannerSubsystem.runOnce(autoTowerCommand));
        NamedCommands.registerCommand("lowShoot",  pathplannerSubsystem.runOnce(lowShootTowerCommand));
        NamedCommands.registerCommand("midShoot",  pathplannerSubsystem.runOnce(midShootTowerCommand));
        NamedCommands.registerCommand("longShoot", pathplannerSubsystem.runOnce(longShootTowerCommand));

        NamedCommands.registerCommand("spinDownTower", pathplannerSubsystem.runOnce(spinDownTowerCommand));
        NamedCommands.registerCommand("spinUpTower",   pathplannerSubsystem.runOnce(spinUpTowerCommand));

        NamedCommands.registerCommand("xModeOn",  pathplannerSubsystem.runOnce(xModeOnCommand));
        NamedCommands.registerCommand("xModeOff", pathplannerSubsystem.runOnce(xModeOffCommand));

        NamedCommands.registerCommand("aimMode",   pathplannerSubsystem.runOnce(autoAimCommand));
        NamedCommands.registerCommand("driveMode", pathplannerSubsystem.runOnce(driveModeCommand));
        NamedCommands.registerCommand("pathplanningMode", pathplannerSubsystem.runOnce(pathplanningCommand));

        NamedCommands.registerCommand("deployIntake", pathplannerSubsystem.runOnce(deployIntakeCommand));
        NamedCommands.registerCommand("retractIntake", pathplannerSubsystem.runOnce(retractIntakeCommand));
        NamedCommands.registerCommand("jiggleIntake", pathplannerSubsystem.runOnce(jiggleIntakeCommand));
    }

    public void Logging()
    {
        Logger.recordOutput("Is Hub Active", isHubActive());
        Logger.recordOutput("Game State", DriverStation.isAutonomous() ? "Autonomous" : DriverStation.isTeleop() && DriverStation.getMatchTime() < 30 ? "EndGame" : "Teleop");

        // What it is

        Logger.recordOutput("Measured/Chassis/Speeds",   m_chassis.getMeasuredSpeeds());
        Logger.recordOutput("Measured/Chassis/States",   m_chassis.getModuleStates());
        Logger.recordOutput("Measured/Chassis/Is Aimed", m_aimController.atSetpoint());

        Logger.recordOutput("Measured/Pose",         m_chassis.getPose());
        Logger.recordOutput("Measured/Heading",      m_chassis.getHeading().getDegrees());
        Logger.recordOutput("Measured/Cam 1 Result", Vision.getResult1());
        Logger.recordOutput("Measured/Cam 2 Result", Vision.getResult2());
        
        Logger.recordOutput("Measured/Chassis To Hub Speed", Units.radiansToDegrees(m_aimController.getError()));

        Logger.recordOutput("Measured/Tower/Is Spun Up", m_tower.isSpunUp());
        Logger.recordOutput("Measured/Tower/TPS",        m_tower.getFlywheelTPS());

        // What we want

        Logger.recordOutput("Desired/Chassis/Speeds", m_speeds);
        Logger.recordOutput("Desired/Chassis/XMode",  m_chassis.getIsXMode());

        Logger.recordOutput("Desired/Tower/TPS",        m_tower.getDesiredFlywheelTPS());
        Logger.recordOutput("Desired/Tower/Manual TPS", m_manualFlywheelSpeed);

        Logger.recordOutput("Desired/Tower/State",   m_towerState.toString() + " " + m_towerRunningState.toString());
        Logger.recordOutput("Desired/Chassis/State", m_chassisState.toString());
        Logger.recordOutput("Desired/Indexer/State", m_indexerState.toString());
        Logger.recordOutput("Desired/Intake/State",  m_intakePosState.toString() + " " + m_intakeRollerState.toString());
        Logger.recordOutput("Desired/Led/State", "No Tyler, there's no LEDs");
    }

    public boolean isHubActive()
    {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty())
        {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled())
        {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled())
        {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime() + 3; // Add 3 seconds of lead time so that we have a competitive edge
        String gameData = DriverStation.getGameSpecificMessage();

        // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        if (gameData.isEmpty()) 
        {
            return true;
        }

        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) 
        {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = 
            switch (alliance.get()) 
            {
                case Red -> !redInactiveFirst;
                case Blue -> redInactiveFirst;
            };

        if (matchTime > 130) 
        {
            // Transition shift, hub is active.
            return true;
        } 
        else if (matchTime > 105) 
        {
            // Shift 1
            return shift1Active;
        } 
        else if (matchTime > 80) 
        {
            // Shift 2
            return !shift1Active;
        } 
        else if (matchTime > 55) 
        {
            // Shift 3
            return shift1Active;
        }
        else if (matchTime > 30) 
        {
            // Shift 4
            return !shift1Active;
        } 
        else 
        {
            // End game, hub always active.
            return true;
        }
    }

    public void Periodic()
    {
        Logging();

        m_manualFlywheelSpeed = SmartDashboard.getNumber("Manual flywheel speed", m_manualFlywheelSpeed);
        // SmartDashboard.putData(manualTowerCommand);
        if (m_towerRunningState == TowerRunningState.Off)
        {
            m_tower.setSpeed(0.0);
        }
        else
        {
            switch (m_towerState)
            {
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
                    double dist = getTargetDistMeters();
                    // TODO:
                    m_tower.setSpeed(0.0);
                    break;
                case ManualControl:
                    m_tower.setSpeed(m_manualFlywheelSpeed);
                    break;
            }
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
            case Pathplanning:
                break;
            case Driving:
                m_chassis.driveFieldRelative(m_speeds);
                break;
            case AimHub:
                chassisTrackAndShootHub.run();
                break;
        }
    }

    public double getTargetDistMeters()
    {
        return Math.sqrt(Math.pow(getTargetPose().getMeasureX().in(Meters) - getTargetPose().getMeasureX().in(Meters), 2) + 
                         Math.pow(getTargetPose().getMeasureY().in(Meters) - getTargetPose().getMeasureY().in(Meters), 2));
    }

    public Pose2d getTargetPose()
    {
        return isHubActive() ?
            ((DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) ? 
                Constants.Field.RedHub.toPose2d() : 
                Constants.Field.BlueHub.toPose2d()) 
            :
            getPose().nearest(Arrays.asList(
                (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) ? Constants.Field.RedAllianceZoneClose : Constants.Field.BlueAllianceZoneClose, 
                (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) ? Constants.Field.RedAllianceZoneFar   : Constants.Field.BlueAllianceZoneFar));
    }

    public Pose2d getPose()
    {
        return m_chassis.getPose();
    }

    public Rotation2d getHeading()
    {
        return m_chassis.getHeading();
    }

    public boolean GetIsReady()
    {
        // If the flywheel is spun up and we're aiming at the target, shoot
        return m_tower.isSpunUp() && 
               m_aimController.atSetpoint() && 
               m_chassisState == ChassisState.AimHub;
    }

    public void setDrive(double leftY, double leftX, double rightX)
    {
        m_speeds = new ChassisSpeeds(
            leftY  * Constants.Chassis.MaximumSpeedMetersPerSec,
            leftX  * Constants.Chassis.MaximumSpeedMetersPerSec, 
            rightX * Constants.Chassis.MaximumAngularVelocity);
    }
}
