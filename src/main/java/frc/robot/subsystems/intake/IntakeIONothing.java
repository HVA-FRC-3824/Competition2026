package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public class IntakeIONothing implements IntakeIO {

  public IntakeIONothing() {

  }

  @Override
  public void setPos(Angle angle) {

  }

  @Override
  public Angle getPos() {

    return Rotations.of(0.0);
  }
}
