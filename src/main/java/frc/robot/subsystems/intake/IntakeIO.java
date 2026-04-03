package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IntakeIO extends Subsystem
{
    public void setPos(double pos);
    public void setRollers(double speed);

   default public boolean isFuelInsideIntake() {
        return false; // True if there is a game piece in the intake
    }

    default public void launchFuel(double shooterSpeed) {}

    default public boolean atSetpoint() { return true; }
}
