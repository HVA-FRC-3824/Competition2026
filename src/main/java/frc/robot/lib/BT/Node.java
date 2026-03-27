package frc.robot.lib.BT;

import edu.wpi.first.wpilibj.XboxController;

public abstract class Node 
{
    abstract NodeStatus update(XboxController controller);

    public enum NodeStatus
    {
        Success,
        Failure
    }
}
