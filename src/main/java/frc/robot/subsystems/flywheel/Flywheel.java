package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.lib.Logged;
import frc.robot.lib.Module;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public abstract class Flywheel extends Module<Flywheel.Inputs, Flywheel.Outputs>
{
  public enum State {
    Backwards,
    Off,
    Low,
    Mid,
    Neutral,
    Field,
    Auto,
    Manual
  }

  public AngularVelocity stateToSpeeds() {
    switch (m_inputs.m_state)
    {
      case Backwards:
        return Constants.Flywheel.CloseSpeed.times(-1.0);
      case Low:
        return Constants.Flywheel.CloseSpeed;
      case Mid:
        return Constants.Flywheel.MiddleSpeed;
      case Neutral:
        return Constants.Flywheel.NeutralPassSpeed;
      case Field:
        return Constants.Flywheel.FieldPassSpeed;
      case Auto:
        return RotationsPerSecond.of(0.166667 * m_inputs.m_distance.in(Inches) + 30.33333);
      case Manual:
        return m_inputs.m_manualVelocity;
      default:
        return RotationsPerSecond.of(0.0);
    }
  }

  public static class Inputs
  {
    public AngularVelocity m_manualVelocity = RotationsPerSecond.of(0.0);
    public State       m_state = State.Off;
    public Distance    m_distance = Meters.of(0.0);
    public double      m_simIntakeSpeed = 0.0; // Balls per sec from intake to flywheel
    public boolean     m_isIndexing = false;

    public Inputs(State state, AngularVelocity manualVelocity, boolean m_canShoot)
    {
      m_state      = state;
      m_manualVelocity = manualVelocity;
    }

    public Inputs(State state, AngularVelocity manualVelocity, double simIntakeSpeed, boolean isIndexing) {
      m_state = state;
      m_manualVelocity = manualVelocity;
      m_simIntakeSpeed = simIntakeSpeed;
      m_isIndexing     = isIndexing;
    }

    public Inputs()
    {

    }
  }

  public static class Outputs extends Logged
  {
    public AngularVelocity m_desiredTPS;
    public AngularVelocity m_measuredTPS;
    public boolean     m_isSpunUp;
    public State       m_state;

    public Outputs()
    {
      m_desiredTPS  = RotationsPerSecond.of(0.0);
      m_measuredTPS = RotationsPerSecond.of(0.0);
      m_isSpunUp  = false;
      m_state     = State.Off;
    }

    public Outputs(AngularVelocity desiredTPS, AngularVelocity measuredTPS, boolean isSpunUp, State state)
    {
      m_desiredTPS  = desiredTPS;
      m_measuredTPS = measuredTPS;
      m_isSpunUp  = isSpunUp;
      m_state     = state;
    }
    
    public void log() {
      Logger.recordOutput("Flywheel/Desired turns per sec",  m_desiredTPS);
      Logger.recordOutput("Flywheel/Measured turns per sec", m_measuredTPS);
      Logger.recordOutput("Flywheel/Is Spun Up",       m_isSpunUp);
      Logger.recordOutput("Flywheel/State",          m_state);
    }
  }

  // Handle logic between subsystems inputs and hardware inputs
  @Override
  protected void updateHardwareInputs() {
    // I'm really not sure why it would ever be null
    if (m_inputs.m_state == null)
      m_inputs.m_state = State.Off;
    
    if (m_inputs.m_state == State.Off) {
      stopFlywheel();
    }

    setFlywheel(stateToSpeeds());
  }

  // Update logging struct (m_outputs)
  @Override
  protected void updateOutputs()
  {
    m_outputs = new Outputs(
      getReference(),
      getMeasured(),
      isSpunUp(),
      m_inputs.m_state
    );
  }

  protected boolean isSpunUp() {
    return getReference().isNear(getMeasured(), Constants.Flywheel.SpunUpTolerance);
        //  && 
        //  getReference().in(RotationsPerSecond) != 0.0;
  }

  protected abstract void setFlywheel(AngularVelocity velocity);

  protected abstract void stopFlywheel();

  protected AngularVelocity getReference() {
    return stateToSpeeds();
  }

  protected abstract AngularVelocity getMeasured();
}
