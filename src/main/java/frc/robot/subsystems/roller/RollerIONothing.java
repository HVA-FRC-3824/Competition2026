package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.AngularVelocity;

public class RollerIONothing implements RollerIO
{
  public RollerIONothing() {

  }

  @Override
  public void setRoller(AngularVelocity velocity) {

  }

  @Override
  public void brakeRoller() {

  }

  @Override
  public AngularVelocity getVelocity() {
    return RotationsPerSecond.of(0.0);
  }
}
