package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.lib.motor.talonFX.SimpleTalon;

public class IntakeSim extends Intake
{
  private SimpleTalon m_motor;

  private DCMotorSim  m_motorModel;
  
  private final IntakeSimulation m_intakeSimulation;

  public IntakeSim(AbstractDriveTrainSimulation driveTrain) {

    m_inputs = new Inputs();
    m_outputs = new Outputs();

    m_intakeSimulation = IntakeSimulation.OverTheBumperIntake(
      "Fuel",
      driveTrain,
      Inches.of(30 - (1.5 * 2)),  // Width
      Inches.of(11.5), // Extension
      IntakeSimulation.IntakeSide.FRONT,
      40 // Capacity
    );

    m_intakeSimulation.addGamePiecesToIntake(8); // Preloads

    m_motor = new SimpleTalon(Constants.CanIds.IntakePositionFollowerMotorId, true); // is an X60

    m_motorModel = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
        DCMotor.getKrakenX60(2), 0.210408789, 1.0 // MOI from CAD
      ),
      DCMotor.getKrakenX60(2)
    );
  }

  public IntakeSimulation getSimIntake() {
    return m_intakeSimulation;
  }

  @Override
  protected void setPos(Angle angle) {

    m_motor.setPosition(angle);

    m_motorModel.setInputVoltage(m_motor.getAppliedVoltage().in(Volts));
    m_motorModel.update(0.02);

    m_motor.simPeriodic(RadiansPerSecond.of(m_motorModel.getAngularVelocityRadPerSec()));

    if (angle.gt(Constants.Intake.IntakeStowedTurns) && m_inputs.m_rollersOn) {
      m_intakeSimulation.startIntake();
    } else {
      m_intakeSimulation.stopIntake();
    }

  }

  @Override
  public Angle getPos()
  {
    return m_motor.getPos();
  }
}
