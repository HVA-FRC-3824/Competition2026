package frc.robot.subsystems.indexer;

import edu.wpi.first.units.measure.AngularVelocity;

public interface IndexerIO {
    
  public void setBelts(AngularVelocity indexerVelocity);
  public void setKicker(AngularVelocity kickerVelocity); 
  public void brakeMotors();
  public AngularVelocity getIndexerVelocity();
  public AngularVelocity getKickerVelocity();
}
