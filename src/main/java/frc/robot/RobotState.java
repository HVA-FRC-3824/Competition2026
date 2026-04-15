package frc.robot;

import static edu.wpi.first.units.Units.Meters;


import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
    import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
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
        Field,
        Neutral,
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

    private static final PIDController m_aimController = new PIDController(3.0, 0.0, 0.5);
    
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
        double time = Timer.getTimestamp();
        time = (time - (double) ((int) time));

        if (time > 0.8)
        {
            m_intakePosState = IntakePosState.Stowed;
        }
        else if (time > 0.55)
        {
            m_intakePosState = IntakePosState.Deployed;
        }
        else if (time > 0.4)
        {
            m_intakePosState = IntakePosState.Stowed;
        }
        else if (time > 0.25)
        {
            m_intakePosState = IntakePosState.Deployed;
        }
        else
        {
            m_intakePosState = IntakePosState.Stowed;
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
    static public final Runnable fieldShootTowerCommand = () -> m_towerState = TowerState.Field;
    static public final Runnable neutralShootTowerCommand = () -> m_towerState = TowerState.Neutral;
    static public final Runnable autoTowerCommand      = () -> m_towerState = TowerState.Auto;
    static public final Runnable manualTowerCommand    = () -> m_towerState = TowerState.ManualControl;

    static public final Runnable increaseManualTowerCommand = () -> m_manualFlywheelSpeed += 1.0;
    static public final Runnable decreaseManualTowerCommand = () -> m_manualFlywheelSpeed -= 1.0;
    static public final Runnable resetManualTowerCommand    = () -> m_manualFlywheelSpeed  = 30.0;

    // Chassis
    static public final Runnable autoAimCommand      = () -> m_chassisState = ChassisState.AimHub;
    static public final Runnable driveModeCommand    = () -> m_chassisState = ChassisState.Driving;
    static public final Runnable pathplanningCommand = () -> m_chassisState = ChassisState.Pathplanning;

    static public final Runnable resetGyro  = () -> m_chassis.resetGyroAngle();

    static public final Runnable xModeCommand    = () -> m_chassis.toggleXMode();
    static public final Runnable xModeOnCommand  = () -> { if (!m_chassis.getIsXMode()) m_chassis.toggleXMode(); };
    static public final Runnable xModeOffCommand = () -> { if (m_chassis.getIsXMode()) m_chassis.toggleXMode(); };
    
    static public final Runnable chassisTrackAndShootHub = () -> {
        Transform2d relativeDistance = getPose().minus(getTargetPose());

        Rotation2d angleToHubFromPos = new Rotation2d(Math.atan2(relativeDistance.getY(), relativeDistance.getX()));

        m_speeds.omegaRadiansPerSecond = m_aimController.calculate(getHeading().getRadians(), angleToHubFromPos.getRadians());

        // if (m_aimController.atSetpoint() && getTargetPose() == ((DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) ? Constants.Field.RedHub : Constants.Field.BlueHub).toPose2d())
        // {
        //     xModeOnCommand.run();
        // }
        // else
        // {
        //     xModeOffCommand.run();
        // }

        m_chassis.driveFieldRelative(m_speeds);
    };



/**
 * A command that does nothing but takes a specified amount of time to finish.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class WaitCommand extends Command {
  /** The timer used for waiting. */
  protected Timer m_timer = new Timer();

  private final double m_duration;

  /**
   * Creates a new WaitCommand. This command will do nothing, and end after the specified duration.
   *
   * @param seconds the time to wait, in seconds
   */
  @SuppressWarnings("this-escape")
  public WaitCommand(double seconds) {
    m_duration = seconds;
    SendableRegistry.setName(this, getName() + ": " + seconds + " seconds");
  }

  /**
   * Creates a new WaitCommand. This command will do nothing, and end after the specified duration.
   *
   * @param time the time to wait
   */
  public WaitCommand(Time time) {
    this(time.in(Seconds));
  }

  @Override
  public void initialize() {
    m_timer.restart();
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_duration);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("duration", () -> m_duration, null);
  }
}


    class PathplannerSubsystem extends SubsystemBase
    {
        public PathplannerSubsystem() {}
    }

    public RobotState()
    {

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
        NamedCommands.registerCommand("fieldShoot", pathplannerSubsystem.runOnce(fieldShootTowerCommand));
        NamedCommands.registerCommand("neutralShoot", pathplannerSubsystem.runOnce(neutralShootTowerCommand));

        NamedCommands.registerCommand("spinDownTower", pathplannerSubsystem.runOnce(spinDownTowerCommand));
        NamedCommands.registerCommand("spinUpTower",   pathplannerSubsystem.runOnce(spinUpTowerCommand));

        NamedCommands.registerCommand("xModeOn",  pathplannerSubsystem.runOnce(xModeOnCommand));
        NamedCommands.registerCommand("wait for bombs",  new WaitCommand(4.5));
        NamedCommands.registerCommand("xModeOff", pathplannerSubsystem.runOnce(xModeOffCommand));

        NamedCommands.registerCommand("aimMode",   pathplannerSubsystem.runOnce(autoAimCommand));
        NamedCommands.registerCommand("driveMode", pathplannerSubsystem.runOnce(driveModeCommand));
        NamedCommands.registerCommand("pathplanningMode", pathplannerSubsystem.runOnce(pathplanningCommand));

        NamedCommands.registerCommand("deployIntake", pathplannerSubsystem.runOnce(deployIntakeCommand).andThen(pathplannerSubsystem.runOnce(startIntakeCommand)));
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

        Logger.recordOutput("Measured/Pose",             m_chassis.getPose());
        Logger.recordOutput("Measured/Dist From Target", Units.metersToInches(getTargetDistMeters()));
        Logger.recordOutput("Measured/Heading",          m_chassis.getHeading().getDegrees());

        Logger.recordOutput("Measured/Chassis To Hub Speed", Units.radiansToDegrees(m_aimController.getError()));
        Logger.recordOutput("Measured/Target Dist Inches", Units.metersToInches(getTargetDistMeters()));

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
                case Field:
                    if (RobotBase.isSimulation() && m_indexerState == IndexerState.Spindexing)
                        m_intake.launchFuel(25.0); // TODO FIX
                    m_tower.setSpeed(Constants.Tower.FieldPassSpeed);
                    break;
                case Neutral:
                    if (RobotBase.isSimulation() && m_indexerState == IndexerState.Spindexing)
                        m_intake.launchFuel(25.0); // TODO FIX
                    m_tower.setSpeed(Constants.Tower.NeutralPassSpeed);
                    break;
                case Auto:
                    double dist = Units.metersToInches(getTargetDistMeters());
                    double desiredGamePieceSpeedFtPerSec = 15 + dist * 0.08; // In my magical perfect world :heart:

                    if (RobotBase.isSimulation() && m_indexerState == IndexerState.Spindexing)
                        m_intake.launchFuel(desiredGamePieceSpeedFtPerSec);

                    m_tower.setSpeed(0.166667 * dist + 30.33333);
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
                break;
            case Driving:
                m_chassis.driveFieldRelative(m_speeds);
                break;
            case AimHub:
                chassisTrackAndShootHub.run();
                break;
        }
        
        m_tower.periodic(); // manually do so bc WPILib command based is very silly and I don't like it
        m_indexer.periodic(); // manually do so bc WPILib command based is very silly and I don't like it
        m_intake.periodic(); // manually do so bc WPILib command based is very silly and I don't like it
        m_chassis.periodic(); // manually do so bc WPILib command based is very silly and I don't like it
    }

    static public double getTargetDistMeters()
    {
        return Math.sqrt(Math.pow(getTargetPose().getMeasureX().in(Meters) - getPose().getMeasureX().in(Meters), 2) + 
                         Math.pow(getTargetPose().getMeasureY().in(Meters) - getPose().getMeasureY().in(Meters), 2));
    }

    static public Pose2d getTargetPose()
    {
        // If were in our alliance zone, score fuel to our alliance zone
        // if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue ?
        //     getPose().getX() > Constants.Field.FieldLengthMeters - Constants.Field.AllianceWallToAllianceZoneMeters :
        //     getPose().getX() < Constants.Field.AllianceWallToAllianceZoneMeters)
        // {
            return ((DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) ? 
                Constants.Field.RedHub.toPose2d() : 
                Constants.Field.BlueHub.toPose2d());
        // }
        // else
        // {
        //     return getPose().nearest(Arrays.asList(
        //         (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) ? Constants.Field.RedAllianceZoneClose : Constants.Field.BlueAllianceZoneClose, 
        //         (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) ? Constants.Field.RedAllianceZoneFar   : Constants.Field.BlueAllianceZoneFar));

        // }
    }

    static public Pose2d getPose()
    {
        return m_chassis.getPose();
    }

    static public Rotation2d getHeading()
    {
        return m_chassis.getHeading();
    }

    static public boolean isSpunUp()
    {
        if (m_tower.getDesiredFlywheelTPS() == 0.0)
        {
            return false;
        }
        
        return m_tower.isSpunUp();
    }

    static public boolean GetIsReady()
    {
        // If the flywheel is spun up and we're aiming at the target, shoot
        return isSpunUp() && 
               m_aimController.atSetpoint() && 
               m_chassisState == ChassisState.AimHub;
    }

    static public void setDrive(double leftY, double leftX, double rightX)
    {
        double angle = Math.atan2(leftY, leftX);

        double magnitude = Math.sqrt(leftY * leftY + leftX * leftX);

        magnitude = Math.pow(Math.abs(magnitude), Constants.Chassis.TranslateExponentialPower) * magnitude; // expo stuff here

        double strafe   = magnitude * Math.sin(angle);
        double forwards = magnitude * Math.cos(angle);

        m_speeds = new ChassisSpeeds(
            strafe  * Constants.Chassis.MaximumSpeedMetersPerSec, 
            forwards  * Constants.Chassis.MaximumSpeedMetersPerSec, 
            rightX * Constants.Chassis.MaximumAngularVelocity);
    }
}
