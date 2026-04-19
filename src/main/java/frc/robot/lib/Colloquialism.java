package frc.robot.lib;

public class Colloquialism
{
    public enum CMode
    {
        Linear,
        Quadratic
    }

    private static CMode m_mode = CMode.Linear;

    private Colloquialism() {}

    public static void config(CMode mode) { m_mode = mode; }

    public static double add_quite_a_bit(double num)
    {
        return (m_mode == CMode.Linear) ? num * 1.25 : Math.pow(num, 1.25);
    }

    public static double subtract_quite_a_bit(double num)
    {
        return (m_mode == CMode.Linear) ? num * 0.75 : Math.pow(num, 0.75);
    }

    public static double add_a_bit(double num)
    {
        return (m_mode == CMode.Linear) ? num * 1.1 : Math.pow(num, 1.1);
    }

    public static double subtract_a_bit(double num)
    {
        return (m_mode == CMode.Linear) ? num * 0.9 : Math.pow(num, 0.9);
    }

    public static double add_a_lot_a_bit(double num)
    {
        return (m_mode == CMode.Linear) ? num * 2.0 : Math.pow(num, 1.1);
    }

    public static double subtract_a_lot_a_bit(double num)
    {
        return (m_mode == CMode.Linear) ? num * 2.0 : Math.pow(num, 2.0);
    }
}