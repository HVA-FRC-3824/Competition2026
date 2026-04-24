package frc.robot.lib.motor.talonFX;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.lib.motor.MotorConfig;
import frc.robot.lib.motor.MotorIO;

public class SimpleTalon implements MotorIO
{

  final TalonFX m_motor;

  final Supplier<Angle>       m_posSupplier;
  final Supplier<AngularVelocity> m_velSupplier;

  final TalonFXSimState m_motorSim;

  Angle m_simPos;

  public SimpleTalon(int id, boolean isX60)
  {
    m_motor = new TalonFX(id);
    m_posSupplier = m_motor.getPosition().asSupplier();
    m_velSupplier = m_motor.getVelocity().asSupplier();

    if (RobotBase.isSimulation())
    {
      m_motorSim = m_motor.getSimState();
      m_motorSim.setMotorType(isX60 ? MotorType.KrakenX60 : MotorType.KrakenX44);

      m_simPos = Rotations.of(0.0);
    }
    else
    {
      m_motorSim = null;
    }
    
    OrchestraOrchestrator.addInstrument(m_motor);
  }

  public SimpleTalon(int id, MotorConfig config, boolean isX60)
  {
    this(id, isX60);

    config.Apply(m_motor);
  }

  public SimpleTalon(int id, MotorConfig config)
  {
    // Assume its an X60, although its not relevant for real life
    // Real implementations for an X44 running this will be fine
    this(id, config, true);
  }

  public SimpleTalon(int id)
  {
    // Assume its an X60, although its not relevant for real life
    // Real implementations for an X44 running this will be fine
    this(id, true);
  }

  public void config(MotorConfig config)
  {
    config.Apply(m_motor);
  }

  // Call this in your updateHardwareInputs function 
  // Get the velocity from your motor model (eg FlywheelSim)
  public void simPeriodic(AngularVelocity velocity)
  {
    m_simPos = m_simPos.plus(Rotations.of((velocity.in(RotationsPerSecond) / 60.0) * 0.02));
    m_motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_motorSim.setRawRotorPosition(m_simPos);
    m_motorSim.setRotorVelocity(velocity.in(RotationsPerSecond));
  }

  public Voltage getAppliedVoltage()
  {
    return RobotBase.isSimulation() ? m_motorSim.getMotorVoltageMeasure() : m_motor.getMotorVoltage().getValue();
  }

  public Voltage getSupplyVoltage()
  {
    return RobotBase.isSimulation() ? Volts.of(RobotController.getBatteryVoltage()) : m_motor.getSupplyVoltage().getValue();
  }

  public Angle getPos()
  {
    return m_posSupplier.get();
  }

  public AngularVelocity getVelocity()
  {
    return m_velSupplier.get();
  }

  public void setVelocity(AngularVelocity angleVel)
  {
    OrchestraOrchestrator.removeInstrument(m_motor.getDeviceID());
    m_motor.setControl(new MotionMagicVelocityVoltage(angleVel));
  }

  public void setPosition(Angle angle)
  {
    OrchestraOrchestrator.removeInstrument(m_motor.getDeviceID());
    m_motor.setControl(new MotionMagicVoltage(angle));
  }

  public void follow(int id, boolean inverted)
  {
    OrchestraOrchestrator.removeInstrument(m_motor.getDeviceID());
    m_motor.setControl(new Follower(id, inverted ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
  }

  public void brake()
  {
    m_motor.setControl(new VoltageOut(0.0));
    // OrchestraOrchestrator.addInstrument(m_motor);
  }

  public void resetEncoder(Angle angle)
  {
    m_motor.setPosition(angle);

    if (RobotBase.isSimulation()) m_simPos = angle;
  }
}
