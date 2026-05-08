package frc.robot.subsystems.flywheel;

import edu.wpi.first.units.measure.AngularVelocity;

public interface FlywheelIO {
    
  public void setFlywheel(AngularVelocity velocity);

  public abstract void stopFlywheel();
  
  public AngularVelocity getMeasured();
}
