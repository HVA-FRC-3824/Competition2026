package frc.robot.subsystems.roller;

import edu.wpi.first.units.measure.AngularVelocity;

public interface RollerIO {
    
  public void setRoller(AngularVelocity velocity); 
  public void brakeRoller();
  public AngularVelocity getVelocity();
}
