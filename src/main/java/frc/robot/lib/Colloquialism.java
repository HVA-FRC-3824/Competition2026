package frc.robot.lib;

public class Colloquialism
{
    public enum CMode
    {
        Linear,
        Quadratic
    }

    private static CMode mode = CMode.Linear;

    private Colloquialism() {}
}