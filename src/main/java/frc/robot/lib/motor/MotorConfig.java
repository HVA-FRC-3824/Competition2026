package frc.robot.lib.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import lombok.Getter;

public class MotorConfig {

  @Getter Current supplyCurrent;
  @Getter Current statorCurrent;
  @Getter boolean inverted;
  @Getter boolean brakeMode;
  @Getter boolean continuousWrap;
  @Getter double P;
  @Getter double I;
  @Getter double D;
  @Getter double S;
  @Getter double V;
  @Getter double A;
  @Getter AngularVelocity     velocityLimit;
  @Getter AngularAcceleration accelerationLimit;
  @Getter double sensorToMechanismRatio;

  public MotorConfig() {
    supplyCurrent  = Amps.of(0.0);
    statorCurrent  = Amps.of(0.0);
    inverted       = false;
    brakeMode      = true;
    continuousWrap = true;
    P = 0.0;
    I = 0.0;
    D = 0.0;
    S = 0.0;
    V = 0.0;
    A = 0.0;
    velocityLimit     = RotationsPerSecond.of(120.0);
    accelerationLimit = RotationsPerSecondPerSecond.of(1200.0);
    sensorToMechanismRatio = 1.0;
  }
  
  public MotorConfig withSupplyLimit(Current limit) {
    supplyCurrent = limit; return this;
  }

  public MotorConfig withStatorLimit(Current limit) {
    statorCurrent = limit; return this;
  }

  public MotorConfig withInverted(boolean inverted) {
    this.inverted = inverted; return this;
  }

  public MotorConfig withBrakeMode(boolean broek) {
    brakeMode = broek; return this;
  }

  public MotorConfig withContinuousWrap(boolean isWrapped) {
    continuousWrap = isWrapped; return this;
  }

  public MotorConfig withP(double p) {
    P = p; return this;

  }

  public MotorConfig withI(double i) {
    I = i; return this;
  }

  public MotorConfig withD(double d) {
    D = d; return this;
  }

  public MotorConfig withS(double s) {
    S = s; return this;
  }

  public MotorConfig withV(double v) {
    V = v; return this;
  }

  public MotorConfig withA(double a) {
    A = a; return this;
  }

  public MotorConfig withVelocityLimit(AngularVelocity limit) {
    velocityLimit = limit; return this;
  }

  public MotorConfig withAccelerationLimit(AngularAcceleration limit) {
    accelerationLimit = limit; return this;
  }

  public MotorConfig withSensorToMechanismRatio(double ratio) {
    sensorToMechanismRatio = ratio; return this;
  }
}
