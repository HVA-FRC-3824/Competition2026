package frc.robot.lib.motor;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface MotorIO
{
    // Call this in your updateHardwareInputs function 
    // Get the velocity from your motor model (eg FlywheelSim)
    default void simPeriodic(AngularVelocity velocity) {}

    void config(MotorConfig config);

    void follow(int id, boolean inverted);
    void brake();
    void setPosition(Angle angle);
    void setVelocity(AngularVelocity angleVel);

    void resetEncoder(Angle angle);

    Voltage getAppliedVoltage();
    Voltage getSupplyVoltage();

    Angle           getPos();
    AngularVelocity getVelocity();
}
