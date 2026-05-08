package frc.robot.subsystems.swerveModule;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    
  public void setState(SwerveModuleInputs state);
  public void brake();
  public SwerveModuleState    getState();
  public SwerveModulePosition getPosition();
  public void resetEncoders();
  public void setWheelAngleToForward();

  public int getNum();
}
