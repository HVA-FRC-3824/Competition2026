package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.Logged;
import frc.robot.lib.Module;
import frc.robot.lib.Util;
import frc.robot.subsystems.belts.Belts;
import frc.robot.subsystems.belts.BeltsSim;
import frc.robot.subsystems.belts.BeltsTalonFX;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelSiim;
import frc.robot.subsystems.flywheel.FlywheelTalonFX;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.gyro.GyroPigeon;
import frc.robot.subsystems.gyro.GyroSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeSim;
import frc.robot.subsystems.intake.IntakeTalonFX;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerSim;
import frc.robot.subsystems.roller.RollerTalonFX;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveReal;
import frc.robot.subsystems.swerve.SwerveSimpleSim;
import frc.robot.subsystems.vision.Vision;

public class RobotState extends Module<RobotState.Inputs, RobotState.Outputs>
{
    // public static enum State {
    //     // Send virtually no signals, just drive
    //     Idle,
    //     // Auto locks to target (normally the hub), points towards it, and fires when ready
    //     AimAndShootToTarget,
    //     // Intaking, ready to shoot & driving
    //     OffenceStance,
    //     // Intake up, Just driving
    //     DefenceStance
    // }

    public static enum State {
        // Send virtually no signals, just drive
        Idle,
        // Auto locks to target (normally the hub), points towards it, and fires when ready
        AimAndShootToTarget,
        // Intaking, ready to shoot & driving
        OffenceStance,
        // Intake up, Just driving
        DefenceStance,
        // Idle + reversed belts, rollers, flywheel (may not be necessary, or may have to be tuned down a LOT)
        JogUnstuck,

        JacksonsCustomPathModeOfDoomAndDespair
    }

    public static class Inputs {
        public State   m_state      = State.Idle;
        public double  m_leftX      = 0.0;
        public double  m_leftY      = 0.0;
        public double  m_rightX     = 0.0;
        public boolean m_isIntaking = false;

        public ChassisSpeeds m_customPathingSpeeds = new ChassisSpeeds();

        public Inputs() {
            
        }
    }

    // TODO: add stuff ¬¬like battery, temps, CAN bus stuff¬¬
    public static class Outputs extends Logged {
        State  m_state = State.Idle;
        Pose2d m_pose  = new Pose2d();

        public Outputs() {

        }

        public Outputs(State state, Pose2d pose)
        {
            m_state = state;
            m_pose  = pose;
        }

        public void log() {
            Logger.recordOutput("State", m_state);
            Logger.recordOutput("Pose",  m_pose);
        }
    }

    // SUBSYSTEM INSTANTATIONS

    static private final Swerve m_swerve  = (RobotBase.isSimulation()) 
        ? new SwerveSimpleSim() //new SwerveSim()
        : new SwerveReal();
    static private final Gyro m_gyro = (RobotBase.isSimulation())
        ? new GyroSim(m_swerve.getSimGyro())
        : new GyroPigeon();
    static private final Roller m_roller = (RobotBase.isSimulation()) 
        ? new RollerSim()
        : new RollerTalonFX();
    static private final Belts m_belts = (RobotBase.isSimulation()) 
        ? new BeltsSim() 
        : new BeltsTalonFX();
    static private final Intake m_intake = (RobotBase.isSimulation()) 
        ? new IntakeSim(m_swerve.getSimSwerve()) 
        : new IntakeTalonFX();
    static private final Flywheel m_flywheel = (RobotBase.isSimulation()) 
        ? new FlywheelSiim(m_swerve.getSimSwerve(), m_intake.getSimIntake()) 
        : new FlywheelTalonFX();
    static private final Vision m_vision = new Vision(m_swerve.getSimSwerve());

    // SUBSYSTEM INPUTS

    static public Swerve.Inputs   m_swerveInputs   = new Swerve.Inputs();
    static public Roller.Inputs   m_rollerInputs   = new Roller.Inputs();
    static public Belts.Inputs    m_beltsInputs    = new Belts.Inputs();
    static public Intake.Inputs   m_intakeInputs   = new Intake.Inputs();
    static public Flywheel.Inputs m_flywheelInputs = new Flywheel.Inputs();
    static public Vision.Inputs   m_visionInputs   = new Vision.Inputs();

    // SUBSYSTEM OUTPUTS

    static private Gyro.Outputs     m_gyroOutputs     = m_gyro.getOutputs();
    static private Swerve.Outputs   m_swerveOutputs   = m_swerve.getOutputs();
    @SuppressWarnings("unused")
    static private Roller.Outputs   m_rollerOutputs   = m_roller.getOutputs();
    static private Belts.Outputs    m_beltsOutputs    = m_belts.getOutputs();
    @SuppressWarnings("unused")
    static private Intake.Outputs   m_intakeOutputs   = m_intake.getOutputs();
    static private Flywheel.Outputs m_flywheelOutputs = m_flywheel.getOutputs();
    static public  Vision.Outputs   m_visionOutputs   = new Vision.Outputs();
    
    class PathplannerSubsystem extends SubsystemBase
    {
        public PathplannerSubsystem() {}
    }

    public RobotState()
    {
        m_inputs = new Inputs();
        m_outputs = new Outputs(State.Idle, new Pose2d());

        // Config PID to be tolerant within 5 degrees

        @SuppressWarnings("unused")
        PathplannerSubsystem pathplannerSubsystem = new PathplannerSubsystem();        

        // This should be in robot container
        // NamedCommands.registerCommand("index", pathplannerSubsystem.runOnce(indexingCommand));
    }

    @Override
    public void updateOutputs()
    {
        // Logger.recordOutput("Is Hub Active", HubActivePeriod.isHubActive());
        // Logger.recordOutput("Game State", DriverStation.isAutonomous() ? "Autonomous" : DriverStation.isTeleop() && DriverStation.getMatchTime() < 30 ? "EndGame" : "Teleop");

        // What it is

        // Logger.recordOutput("Measured/Chassis/Speeds",   m_swerve.getMeasuredSpeeds());

        // What we want

        // Logger.recordOutput("Desired/Chassis/Speeds", m_speeds);

        // Simulation

        // Logger.recordOutput("Simulation/Is Fuel In Intake", m_intake.isFuelInsideIntake());

        m_outputs.m_pose  = m_swerveOutputs.m_pose;
        m_outputs.m_state = m_inputs.m_state;
    }

    @Override
    public void updateHardwareInputs() {        
        // manually call periodics so bc WPILib command based is very silly and I don't like it
        m_flywheel.modulePeriodic();
        m_belts.modulePeriodic();
        m_intake.modulePeriodic();
        m_swerve.modulePeriodic();
        m_roller.modulePeriodic();
        m_vision.modulePeriodic();

        m_gyroOutputs     = m_gyro.getOutputs();
        m_swerveOutputs   = m_swerve.getOutputs();
        m_rollerOutputs   = m_roller.getOutputs();
        m_beltsOutputs    = m_belts.getOutputs();
        m_intakeOutputs   = m_intake.getOutputs();
        m_flywheelOutputs = m_flywheel.getOutputs();
        m_visionOutputs   = m_vision.getOutputs();

        m_swerveInputs   = new Swerve.Inputs();
        m_rollerInputs   = new Roller.Inputs();
        m_beltsInputs    = new Belts.Inputs();
        m_intakeInputs   = new Intake.Inputs();
        m_flywheelInputs = new Flywheel.Inputs();
        m_visionInputs   = new Vision.Inputs();

        // Inter-Subsystem Communication
        m_swerveInputs.m_heading           = m_gyroOutputs.m_heading;
        m_swerveInputs.m_visionMeasurement = m_visionOutputs.m_measurements;
        m_intakeInputs.m_rollersOn         = m_beltsOutputs.m_state == Belts.State.On;
        m_flywheelInputs.m_distance        = Util.dist(getPose(), m_swerve.getTargetPose());

        // Controller Inputs
        setDrive();
        switch (m_inputs.m_state) {
            case Idle:
                m_swerveInputs.m_state   = Swerve.State.FieldRelativeDriving;
                m_rollerInputs.m_state   = Roller.State.Off;
                m_flywheelInputs.m_state = Flywheel.State.Off;
                // I would rather not move intake if I can
                if (m_intakeInputs.m_state == Intake.State.Alligator)
                    m_intakeInputs.m_state = Intake.State.Stowed;
                m_flywheelInputs.m_state = Flywheel.State.Off;
                break;
            case AimAndShootToTarget:
                m_swerveInputs.m_state   = Swerve.State.Aiming;
                m_flywheelInputs.m_state = Flywheel.State.Mid;
                boolean isready = GetIsReady();
                m_beltsInputs.m_state  = isready ? Belts.State.On : Belts.State.Off;
                m_rollerInputs.m_state = isready ? Roller.State.On : Roller.State.Off;
                m_intakeInputs.m_state = isready ? Intake.State.Alligator : Intake.State.Deployed;
                break;
            case OffenceStance:
                m_swerveInputs.m_state = Swerve.State.FieldRelativeDriving;
                m_intakeInputs.m_state = Intake.State.Deployed;
                m_rollerInputs.m_state = m_inputs.m_isIntaking ? Roller.State.On : Roller.State.Off;
                break;
            case DefenceStance:
                m_rollerInputs.m_state   = Roller.State.Off;
                m_flywheelInputs.m_state = Flywheel.State.Off;
                m_intakeInputs.m_state   = Intake.State.Starting;
                break;
            case JogUnstuck:
                m_beltsInputs.m_state    = Belts.State.Backwards;
                m_flywheelInputs.m_state = Flywheel.State.Backwards;
                break;
            case JacksonsCustomPathModeOfDoomAndDespair:
                m_swerveInputs.m_speeds = m_inputs.m_customPathingSpeeds;
                m_swerveInputs.m_state  = Swerve.State.FieldRelativeDriving;
                break;
        }

        m_swerve.setInputs(m_swerveInputs);
        m_roller.setInputs(m_rollerInputs);
        m_belts.setInputs(m_beltsInputs);
        m_intake.setInputs(m_intakeInputs);
        m_flywheel.setInputs(m_flywheelInputs);
    }

    public Pose2d getPose()
    {
        return m_swerveOutputs.m_pose;
    }

    public Rotation2d getHeading()
    {
        return new Rotation2d(m_gyroOutputs.m_heading);
    }

    public boolean GetIsReady()
    {
        // If the flywheel is spun up and we're aiming at the target, shoot
        return m_flywheelOutputs.m_isSpunUp && 
               m_swerveOutputs.m_isAimed && 
               m_swerveInputs.m_state == Swerve.State.Aiming;
    }

    public void setDrive()
    {
        double angle = Math.atan2(m_inputs.m_leftY, m_inputs.m_leftX);
        double magnitude = Math.sqrt(m_inputs.m_leftY * m_inputs.m_leftY + m_inputs.m_leftX * m_inputs.m_leftX);

        magnitude = Math.pow(Math.abs(magnitude), Constants.Chassis.TranslateExponentialPower) * magnitude; // expo stuff here

        double strafe   = magnitude * Math.sin(angle);
        double forwards = magnitude * Math.cos(angle);

        double rotation = Math.pow(Math.abs(m_inputs.m_rightX), Constants.Chassis.AngularExponentialPower) * m_inputs.m_rightX;

        m_swerveInputs.m_speeds = new ChassisSpeeds(
            Constants.Chassis.MaximumLinear.times(strafe), 
            Constants.Chassis.MaximumLinear.times(forwards), 
            Constants.Chassis.MaximumAngularVelocity.times(rotation));
    }

    public Swerve getSwerveRefForPathing() {
        return m_swerve;
    }
}
