package frc3824;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Meters;

import org.dyn4j.geometry.Vector2;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc3824.Constants;
import frc3824.RobotState.IndexerState;
import frc3824.subsystems.Chassis;
import frc3824.subsystems.Indexer;
import frc3824.subsystems.Intake;
import frc3824.subsystems.Tower;

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

    private static final Tower       m_tower   = new Tower();
    private static final Indexer     m_indexer = new Indexer();
    private static final Intake      m_intake  = new Intake();
    private static final Chassis     m_chassis = new Chassis();

    private static final PIDController m_aimController = new PIDController(0.5, 0.0, 0.0);

    // Indexer
    public final Command indexingCommand          = new InstantCommand(() -> { m_indexerState = IndexerState.Spindexing; }, m_indexer);
    public final Command notIndexingCommand       = new InstantCommand(() -> { m_indexerState = IndexerState.Stopped; },    m_indexer);
    public final Command backwardsIndexingCommand = new InstantCommand(() -> { m_indexerState = IndexerState.Backwards; },  m_indexer);

    // Intake
    public final Command deployIntakeCommand   = new InstantCommand(() -> { m_intakePosState = IntakePosState.Deployed; }, m_intake);
    public final Command retractIntakeCommand  = new InstantCommand(() -> { m_intakePosState = IntakePosState.Stowed; },   m_intake);

    public final Command startIntakeCommand    = new InstantCommand(() -> { m_intakeRollerState = IntakeRollerState.Intaking; },  m_intake);
    public final Command stopIntakeCommand     = new InstantCommand(() -> { m_intakeRollerState = IntakeRollerState.Off; },       m_intake);
    public final Command spitOutIntakeCommand  = new InstantCommand(() -> { m_intakeRollerState = IntakeRollerState.Backwards; }, m_intake);

    // Tower
    public final Command spinDownTowerCommand       = new InstantCommand(() -> { m_towerState = TowerState.Idle; },          m_tower);
    public final Command lowShootTowerCommand       = new InstantCommand(() -> { m_towerState = TowerState.Low; },           m_tower);
    public final Command midShootTowerCommand       = new InstantCommand(() -> { m_towerState = TowerState.Middle; },        m_tower);
    public final Command longShootTowerCommand      = new InstantCommand(() -> { m_towerState = TowerState.Long; },          m_tower);
    public final Command autoTowerCommand           = new InstantCommand(() -> { m_towerState = TowerState.Auto; },          m_tower);
    public final Command manualTowerCommand         = new InstantCommand(() -> { m_towerState = TowerState.ManualControl; }, m_tower);
    public final Command increaseManualTowerCommand = new InstantCommand(() -> { m_manualFlywheelSpeed += 5.0; }, m_tower);


    // Chassis
    public final Command autoAimCommand          = new InstantCommand(() -> { m_chassisState = ChassisState.AimHub; }, m_chassis);
    public final Command driveModeCommand        = new InstantCommand(() -> { m_chassisState = ChassisState.AimHub; }, m_chassis);

    public final Command xModeCommand            = new InstantCommand(() -> { m_chassis.ToggleXMode(); }, m_chassis);
    public final Command xModeOnCommand          = new InstantCommand(() -> { if (!m_chassis.GetIsXMode()) m_chassis.ToggleXMode(); }, m_chassis);
    public final Command xModeOffCommand         = new InstantCommand(() -> { if (!m_chassis.GetIsXMode()) m_chassis.ToggleXMode(); }, m_chassis);
    
    public final Command slowModeCommand         = new InstantCommand(() -> { m_chassis.ToggleSlowMode(); }, m_chassis);
    public final Command slowModeOnCommand       = new InstantCommand(() -> { if (!m_chassis.GetIsSlowMode()) m_chassis.ToggleSlowMode(); }, m_chassis);
    public final Command slowModeOffCommand      = new InstantCommand(() -> { if (!m_chassis.GetIsSlowMode()) m_chassis.ToggleSlowMode(); }, m_chassis);
    
    public final Command fieldRelativeCommand    = new InstantCommand(() -> { m_chassis.ToggleXMode(); }, m_chassis);
    public final Command fieldRelativeOnCommand  = new InstantCommand(() -> { if (!m_chassis.GetIsFieldRelative()) m_chassis.ToggleFieldCentric(); }, m_chassis);
    public final Command fieldRelativeOffCommand = new InstantCommand(() -> { if (!m_chassis.GetIsFieldRelative()) m_chassis.ToggleFieldCentric(); }, m_chassis);


    private static final RobotState state = new RobotState();
    public static RobotState GetInstance()
    {
        return state;
    }

    private RobotState()
    {
        // Config PID to be tolerant within 5 degrees
        m_aimController.setTolerance(Units.degreesToRadians(5.0));

        // This should be in robot container
        NamedCommands.registerCommand("Index", indexingCommand);
        NamedCommands.registerCommand("Stop Indexing", notIndexingCommand);

        NamedCommands.registerCommand("stopShoot", spinDownTowerCommand);
        NamedCommands.registerCommand("lowShoot", lowShootTowerCommand);
        NamedCommands.registerCommand("midShoot", midShootTowerCommand);
        NamedCommands.registerCommand("longShoot", longShootTowerCommand);

        NamedCommands.registerCommand("xModeOn", xModeOnCommand);
        NamedCommands.registerCommand("xModeOff", xModeOffCommand);

        NamedCommands.registerCommand("aimmode", autoAimCommand);
        NamedCommands.registerCommand("driveMode", driveModeCommand);
    }

    public void Periodic()
    {
        m_manualFlywheelSpeed = SmartDashboard.getNumber("Manual flywheel speed", m_manualFlywheelSpeed);
        SmartDashboard.putData(manualTowerCommand);

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
                m_indexer.SetSpeeds(0.0, 0.0);
                break;
            case Spindexing:
                m_indexer.SetSpeeds(Constants.Indexer.BeltTurnsPerSec, Constants.Indexer.KickerWheelTurnsPerSec);
                break;
            case Backwards:
                m_indexer.SetSpeeds(-Constants.Indexer.BeltTurnsPerSec, -Constants.Indexer.KickerWheelTurnsPerSec);
                break;
        }
    
        switch (m_intakePosState)
        {
            case Stowed:
                m_intake.setPos(Constants.Intake.IntakeStowedDegrees);
                break;
            case Deployed:
                m_intake.setPos(Constants.Intake.IntakeDeployedDegrees);
                break;
            case StartingPos:
                SmartDashboard.putString("Why ", "are you going to the starting pos");
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

        if (DriverStation.isTeleop())
        {
            switch (m_chassisState)
            {
                case Driving:
                    m_chassis.Drive(m_speeds);
                    break;
                case AimHub:
                    Transform2d relativeDistance = GetHubPose().minus(GetPose());

                    Rotation2d angleToHubFromPos = new Rotation2d(Math.atan2(relativeDistance.getY(), relativeDistance.getX()));

                    m_speeds.omegaRadiansPerSecond = m_aimController.calculate(GetHeading().getDegrees(), angleToHubFromPos.getDegrees());

                    if (m_aimController.atSetpoint())
                        CommandScheduler.getInstance().schedule(xModeOnCommand);                        
                    else
                        CommandScheduler.getInstance().schedule(xModeOffCommand);

                    m_chassis.Drive(m_speeds);
                    break;
            }
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
                        Constants.Field.RedHub : Constants.Field.BlueHub;
    }

    public Pose2d GetPose()
    {
        return m_chassis.GetPose();
    }

    public Rotation2d GetHeading()
    {
        return m_chassis.GetHeading();
    }

    public Boolean GetIsReady()
    {
        // If the flywheel is spun up and we're aiming at the target, shoot
        return m_tower.isSpunUp() && m_aimController.atSetpoint();
    }

    public void setDrive(double leftY, double leftX, double rightX)
    {
        m_speeds = new ChassisSpeeds(leftY  * Constants.Chassis.MaximumSpeedMetersPerSec, 
                                     leftX  * Constants.Chassis.MaximumSpeedMetersPerSec,
                                     rightX * Constants.Chassis.MaximumAngularVelocity);
    }
}
