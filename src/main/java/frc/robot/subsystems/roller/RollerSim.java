package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.lib.motor.io.TalonIO;

public class RollerSim implements RollerIO
{
  public TalonIO m_motor;

  public DCMotorSim  m_motorModel;
  
  private final IntakeSimulation m_intakeSimulation;

  public RollerSim(AbstractDriveTrainSimulation driveTrain) {
    
    m_motor = new TalonIO(Constants.CanIds.FuelIntakeMotorId, Constants.Roller.RollerConfig, true); // is an X60

    m_motorModel = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
        DCMotor.getKrakenX60(1), 0.01102666212, 1.0 // MOI from CAD
      ),
      DCMotor.getKrakenX60(1)
    );

    
    m_intakeSimulation = IntakeSimulation.OverTheBumperIntake(
      "Fuel",
      driveTrain,
      Inches.of(30 - (1.5 * 2)),  // Width
      Inches.of(11.5), // Extension
      IntakeSimulation.IntakeSide.FRONT,
      40 // Capacity
    );

    m_intakeSimulation.addGamePiecesToIntake(8); // Preloads
  }

  @Override
  public void setRoller(AngularVelocity velocity)
  {
    m_motor.setVelocity(velocity);

    m_motorModel.setInputVoltage(m_motor.getAppliedVoltage().in(Volts));
    m_motorModel.update(0.02);

    m_motor.simPeriodic(m_motorModel.getAngularVelocity());

    
    if (velocity.gt(RotationsPerSecond.of(1.0))) {
      m_intakeSimulation.startIntake();
    } else {
      m_intakeSimulation.stopIntake();
    }
  }

  @Override
  public void brakeRoller()
  {
    m_motor.brake();
    
    m_motorModel.setInputVoltage(m_motor.getAppliedVoltage().in(Volts));
    m_motorModel.update(0.02);

    m_motor.simPeriodic(m_motorModel.getAngularVelocity());
  }

  @Override
  public AngularVelocity getVelocity()
  {
    return m_motor.getVelocity();
  }
}
