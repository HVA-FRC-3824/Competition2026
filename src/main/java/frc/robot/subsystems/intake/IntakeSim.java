package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.lib.motor.talonFX.SimpleTalon;

public class IntakeSim extends Intake
{
    private SimpleTalon m_motor;

    private SingleJointedArmSim  m_motorModel;
    private Angle                m_lastInput = Rotations.of(0.0);
    
    private final IntakeSimulation m_intakeSimulation;

    public IntakeSim(AbstractDriveTrainSimulation driveTrain) {

        m_inputs = new Inputs();
        m_outputs = new Outputs();

        m_intakeSimulation = IntakeSimulation.OverTheBumperIntake(
            // Specify the type of game pieces that the intake can collect
            "Fuel",
            // Specify the drivetrain to which this intake is attached
            driveTrain,
            Inches.of(30 - (1.5 * 2)),        // Width of the intake
            Inches.of(12.0),        // How far the intake extends beyond the bumper
            IntakeSimulation.IntakeSide.FRONT, // Which side of the chassis the intake is on
            40                        // Capacity: max game pieces the intake can hold
        );

        m_intakeSimulation.addGamePiecesToIntake(8); // Preloads

        m_motor = new SimpleTalon(Constants.CanIds.IntakePositionFollowerMotorId, true); // is an X60

        m_motorModel = new SingleJointedArmSim(
            DCMotor.getKrakenX60(2),
            25, // 2 5s 
            0.210408789,
            0.3302, 
            Math.PI / 2, // min 90 stowed
            Math.PI, // max 180 deployed
            true, 
            Math.PI / 2);
    }

    public IntakeSimulation getSimIntake() {
        return m_intakeSimulation;
    }

    @Override
    protected void setPos(Angle angle)
    {
        m_lastInput = angle;

        m_motor.setPosition(angle);

        m_motorModel.setInputVoltage(m_motor.getAppliedVoltage().in(Volts));
        m_motorModel.update(0.02);

        m_motor.simPeriodic(RadiansPerSecond.of(m_motorModel.getVelocityRadPerSec()));

        if (angle.gt(Constants.Intake.IntakeStowedTurns) && m_inputs.m_rollersOn) 
        {
            m_intakeSimulation.startIntake();
        }
        else
        {
            m_intakeSimulation.stopIntake();
        }


    }

    @Override
    public Angle getPos()
    {
        return m_motor.getPos();
    }
    
    @Override
    public Angle getReference()
    {
        return m_lastInput;
    }
}
