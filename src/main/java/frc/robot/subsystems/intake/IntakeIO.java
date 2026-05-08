package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;

public interface IntakeIO {

  public void setPos(Angle angle); 
  public Angle getPos();
}
