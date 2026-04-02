package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.HubActivePeriod;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.ChassisIO;
import frc.robot.subsystems.chassis.SimChassis;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeSim;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.tower.TowerIO;
import frc.robot.subsystems.tower.TowerSim;

public class RobotState 
{
    public enum MonolithState
    {
        HubInactivePassing,
        HubInactiveIntaking,
        HubInactiveWarmUp,

        HubActiveShooting,
        HubActiveIntaking
    }

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
        Slow,
        AimHub
    }

    // STATES

    static public TowerState        m_towerState = TowerState.Middle;
    static public TowerRunningState m_towerRunningState = TowerRunningState.Off;
    static public IndexerState      m_indexerState = IndexerState.Stopped;
    static public LedState          m_ledState = LedState.MatchMode;
    static public IntakePosState    m_intakePosState = IntakePosState.Stowed;
    static public IntakeRollerState m_intakeRollerState = IntakeRollerState.Off;
    static public double            m_manualFlywheelSpeed = 55.0;
    static public ChassisState      m_chassisState = ChassisState.Driving;
    static public ChassisSpeeds     m_speeds = new ChassisSpeeds();

    // SUBSYSTEM INSTANTATIONS

    static private final TowerIO     m_tower   = (RobotBase.isSimulation()) ? new TowerSim() : new Tower();
    static private final ChassisIO   m_chassis = (RobotBase.isSimulation()) ? new SimChassis() : new Chassis();
    static private final Indexer     m_indexer = new Indexer();
    static private final IntakeIO    m_intake  = (RobotBase.isSimulation()) ? new IntakeSim(m_chassis.getSimChassis()) : new Intake();

    // CONTROLLERS

    private static final PIDController m_aimController = new PIDController(10.0, 0.05, 0.1);
    
    // FUNCTIONS

    // Indexer
    static public final Runnable indexingCommand          = () -> m_indexerState = IndexerState.Spindexing;
    static public final Runnable notIndexingCommand       = () -> m_indexerState = IndexerState.Stopped;
    static public final Runnable backwardsIndexingCommand = () -> m_indexerState = IndexerState.Backwards;
    static public final Runnable jiggleIndexingCommand  = () -> {
        if (((int)Timer.getTimestamp() / 2) % 2 == 0)
        {
            m_indexerState = IndexerState.Spindexing;
        }
        else
        {
            m_indexerState = IndexerState.Backwards;
        }
    };

    // Intake
    static public final Runnable deployIntakeCommand  = () -> m_intakePosState = IntakePosState.Deployed;
    static public final Runnable retractIntakeCommand = () -> m_intakePosState = IntakePosState.Stowed;
    static public final Runnable jiggleIntakeCommand  = () -> {
        if (((int)Timer.getTimestamp() / 2) % 2 == 0)
        {
            m_intakePosState = IntakePosState.Stowed;
        }
        else
        {
            m_intakePosState = IntakePosState.Deployed;
        }
    };

    static public final Runnable startIntakeCommand   = () -> m_intakeRollerState = IntakeRollerState.Intaking;
    static public final Runnable stopIntakeCommand    = () -> m_intakeRollerState = IntakeRollerState.Off;
    static public final Runnable spitOutIntakeCommand = () -> m_intakeRollerState = IntakeRollerState.Backwards;
    
    // Tower
    static public final Runnable spinDownTowerCommand = () -> m_towerRunningState = TowerRunningState.Off;
    static public final Runnable spinUpTowerCommand   = () -> m_towerRunningState = TowerRunningState.On;

    static public final Runnable lowShootTowerCommand  = () -> m_towerState = TowerState.Low;
    static public final Runnable midShootTowerCommand  = () -> m_towerState = TowerState.Middle;
    static public final Runnable longShootTowerCommand = () -> m_towerState = TowerState.Long;
    static public final Runnable autoTowerCommand      = () -> m_towerState = TowerState.Auto;
    static public final Runnable manualTowerCommand    = () -> m_towerState = TowerState.ManualControl;

    static public final Runnable increaseManualTowerCommand = () -> m_manualFlywheelSpeed += 1.0;
    static public final Runnable decreaseManualTowerCommand = () -> m_manualFlywheelSpeed -= 1.0;
    static public final Runnable resetManualTowerCommand    = () -> m_manualFlywheelSpeed  = 30.0;

    // Chassis
    static public final Runnable autoAimCommand      = () -> m_chassisState = ChassisState.AimHub;
    static public final Runnable slowModeCommand     = () -> m_chassisState = ChassisState.Slow;
    static public final Runnable driveModeCommand    = () -> m_chassisState = ChassisState.Driving;
    static public final Runnable pathplanningCommand = () -> m_chassisState = ChassisState.Pathplanning;

    static public final Runnable xModeCommand    = () -> m_chassis.toggleXMode();
    static public final Runnable xModeOnCommand  = () -> { if (!m_chassis.getIsXMode()) m_chassis.toggleXMode(); };
    static public final Runnable xModeOffCommand = () -> { if (m_chassis.getIsXMode()) m_chassis.toggleXMode(); };
    
    static public final Runnable chassisTrackAndShootHub = () -> {
        Transform2d relativeDistance = getPose().minus(getTargetPose());

        Rotation2d angleToHubFromPos = new Rotation2d(Math.atan2(relativeDistance.getY(), relativeDistance.getX()));

        m_speeds.omegaRadiansPerSecond = m_aimController.calculate(getHeading().getRadians(), angleToHubFromPos.getRadians());

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

        m_tower.setDefaultCommand(m_tower.runOnce(()->{}));
        m_indexer.setDefaultCommand(m_indexer.runOnce(()->{}));
        m_intake.setDefaultCommand(m_intake.runOnce(()->{}));
        m_chassis.setDefaultCommand(m_chassis.runOnce(()->{}));

        // Config PID to be tolerant within 5 degrees
        m_aimController.setTolerance(Units.degreesToRadians(2.0));

        m_aimController.enableContinuousInput(-Math.PI, Math.PI);

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

    static public void Logging()
    {
        Logger.recordOutput("Is Hub Active", HubActivePeriod.isHubActive());
        Logger.recordOutput("Game State", DriverStation.isAutonomous() ? "Autonomous" : DriverStation.isTeleop() && DriverStation.getMatchTime() < 30 ? "EndGame" : "Teleop");

        // What it is

        Logger.recordOutput("Measured/Chassis/Speeds",   m_chassis.getMeasuredSpeeds());
        Logger.recordOutput("Measured/Chassis/States",   m_chassis.getModuleStates());
        Logger.recordOutput("Measured/Chassis/Is Aimed", m_aimController.atSetpoint());

        Logger.recordOutput("Measured/Pose",         m_chassis.getPose());
        Logger.recordOutput("Measured/Heading",      m_chassis.getHeading().getDegrees());

        Logger.recordOutput("Measured/Chassis To Hub Speed", Units.radiansToDegrees(m_aimController.getError()));
        Logger.recordOutput("Measured/Target Dist Inches", Units.metersToInches(getTargetDistMeters()));

        Logger.recordOutput("Measured/Tower/Is Spun Up", m_tower.isSpunUp());
        Logger.recordOutput("Measured/Tower/TPS",        m_tower.getFlywheelTPS());

        // What we want

        Logger.recordOutput("Desired/Chassis/Speeds", m_speeds);
        Logger.recordOutput("Desired/Chassis/XMode",  m_chassis.getIsXMode());
        Logger.recordOutput("Desired/Chassis/XMode",  m_chassisState == ChassisState.Slow);

        Logger.recordOutput("Desired/Tower/TPS",        m_tower.getDesiredFlywheelTPS());
        Logger.recordOutput("Desired/Tower/Manual TPS", m_manualFlywheelSpeed);

        Logger.recordOutput("Desired/Tower/State",   m_towerState.toString() + " " + m_towerRunningState.toString());
        Logger.recordOutput("Desired/Chassis/State", m_chassisState.toString());
        Logger.recordOutput("Desired/Indexer/State", m_indexerState.toString());
        Logger.recordOutput("Desired/Intake/State",  m_intakePosState.toString() + " " + m_intakeRollerState.toString());
        Logger.recordOutput("Desired/Led/State", "No Tyler, there's no LEDs");

        // Simulation

        Logger.recordOutput("Simulation/Is Fuel In Intake", m_intake.isFuelInsideIntake());
    }

    static public void Periodic()
    {
        Logging();

        m_aimController.calculate(getHeading().getRadians());

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
                case Low: // 70 in
                    if (RobotBase.isSimulation() && m_indexerState == IndexerState.Spindexing)
                        m_intake.launchFuel(20.6);
                    m_tower.setSpeed(Constants.Tower.CloseSpeed);
                    break;
                case Middle: // 95 in
                    if (RobotBase.isSimulation() && m_indexerState == IndexerState.Spindexing)
                        m_intake.launchFuel(22.6);
                    m_tower.setSpeed(Constants.Tower.MiddleSpeed);
                    break;
                case Long: // 120 in
                    if (RobotBase.isSimulation() && m_indexerState == IndexerState.Spindexing)
                        m_intake.launchFuel(25.0);
                    m_tower.setSpeed(Constants.Tower.LongSpeed);
                    break;
                case Auto:
                    double dist = getTargetDistMeters();
                    double desiredGamePieceSpeedFtPerSec = 15 + dist * 0.08;

                    if (RobotBase.isSimulation() && m_indexerState == IndexerState.Spindexing)
                        m_intake.launchFuel(desiredGamePieceSpeedFtPerSec);
                    // TODO:
                    m_tower.setSpeed(-0.017 + desiredGamePieceSpeedFtPerSec*1.25275);
                    break;
                case ManualControl:
                    if (RobotBase.isSimulation() && m_indexerState == IndexerState.Spindexing)
                        m_intake.launchFuel(20.6);
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
                xModeOffCommand.run();
                break;
            case Driving:
                m_chassis.driveFieldRelative(m_speeds);
                xModeOffCommand.run();
                break;
            case AimHub:
                chassisTrackAndShootHub.run();
                break;
            case Slow:
                m_chassis.driveFieldRelative(new ChassisSpeeds(m_speeds.vxMetersPerSecond / 2, m_speeds.vyMetersPerSecond / 2, m_speeds.omegaRadiansPerSecond / 2));
                xModeOffCommand.run();
                break;
        }
    }

    static public double getTargetDistMeters()
    {
        return Math.sqrt(Math.pow(getTargetPose().getMeasureX().in(Meters) - getTargetPose().getMeasureX().in(Meters), 2) + 
                         Math.pow(getTargetPose().getMeasureY().in(Meters) - getTargetPose().getMeasureY().in(Meters), 2));
    }

    static public Pose2d getTargetPose()
    {
        return HubActivePeriod.isHubActive() ?
            ((DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) ? 
                Constants.Field.RedHub.toPose2d() : 
                Constants.Field.BlueHub.toPose2d()) 
            :
            getPose().nearest(Arrays.asList(
                (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) ? Constants.Field.RedAllianceZoneClose : Constants.Field.BlueAllianceZoneClose, 
                (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) ? Constants.Field.RedAllianceZoneFar   : Constants.Field.BlueAllianceZoneFar));
    }

    static public Pose2d getPose()
    {
        return m_chassis.getPose();
    }

    static public Rotation2d getHeading()
    {
        return m_chassis.getHeading();
    }

    static public boolean GetIsReady()
    {
        // If the flywheel is spun up and we're aiming at the target, shoot
        return m_tower.isSpunUp() && 
               m_aimController.atSetpoint() && 
               m_chassisState == ChassisState.AimHub;
    }

    static public void setDrive(double leftY, double leftX, double rightX)
    {
        // leftY = m_yAccelLimiter.calculate(leftY);
        // leftX = m_xAccelLimiter.calculate(leftX);

        // rightX = Math.pow(Math.abs(rightX), Constants.Chassis.AngularExponentialPower) * rightX;

        m_speeds = new ChassisSpeeds(
            leftY  * Constants.Chassis.MaximumSpeedMetersPerSec,
            leftX  * Constants.Chassis.MaximumSpeedMetersPerSec, 
            rightX * Constants.Chassis.MaximumAngularVelocity);
    }
}
