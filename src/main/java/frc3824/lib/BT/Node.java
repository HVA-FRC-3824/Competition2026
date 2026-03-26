package frc3824.lib.BT;

public abstract class Node 
{
    abstract NodeStatus update();

    public enum NodeStatus
    {
        Success,
        Failure
    }
}
