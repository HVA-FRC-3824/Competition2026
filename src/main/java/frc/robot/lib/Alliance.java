package frc.robot.lib;

import edu.wpi.first.wpilibj.DriverStation;

// This logic is always so ugly and cluttering
// Abstract to here.
public class Alliance {
    private static boolean m_isRed = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red;
    
    public static boolean isRed() { return m_isRed; }
}
