package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class IndexerIONothing implements IndexerIO {

  public IndexerIONothing() {

  }

  @Override
  public void setBelts(AngularVelocity indexerVelocity) {

  }

  @Override
  public void setKicker(AngularVelocity kickerVelocity) {

  }

  @Override
  public void brakeMotors() {

  }

  @Override
  public AngularVelocity getIndexerVelocity() {
    return RotationsPerSecond.of(0.0);
  }

  @Override
  public AngularVelocity getKickerVelocity() {
    return RotationsPerSecond.of(0.0);

  }
}
