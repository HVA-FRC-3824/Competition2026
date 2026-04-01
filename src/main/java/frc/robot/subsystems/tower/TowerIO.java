package frc.robot.subsystems.tower;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public interface TowerIO extends Subsystem
{
    public void setSpeed(double speed);

    public double getDesiredFlywheelTPS();
    public double getFlywheelTPS();

    default public boolean isSpunUp()
    {
        return Math.abs(getFlywheelTPS() - getDesiredFlywheelTPS()) < Constants.Tower.SpunUpTolerance;
    }
}
