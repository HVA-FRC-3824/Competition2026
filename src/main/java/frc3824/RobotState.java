package frc3824;
public class RobotState {
    public enum TowerState
    {
        Idle,
        ShootingToHub,
        PassingToAdjacentZone,
        ManualControl,
        Automatic
    }

    public enum SpindexerState
    {
        Stopped,
        Spindexing,
        Backwards
    }

    public enum LedState
    {
        Off,
        SolidGreen,
        SolidRed,
        HvaColors,
        Strobe,
        Rainbow,
        MatchMode
    }

    enum IntakePosState
    {
        Stowed,
        Deployed,
        Backwards,
        StartingPos
    }

    enum IntakeRollerState
    {
        Intaking,
        Off,
        Backwards
    }

    public TowerState        m_towerState;
    public SpindexerState    m_spindexerState;
    public LedState          m_ledState;
    public IntakePosState    m_intakePosState;
    public IntakeRollerState m_intakeRollerState;

    public RobotState()
    {
        m_towerState = TowerState.Idle;
        m_spindexerState = SpindexerState.Stopped;
        m_ledState = LedState.MatchMode;
        m_intakePosState = IntakePosState.StartingPos;
        m_intakeRollerState = IntakeRollerState.Off;
    }

    public void Periodic()
    {
        
    }
}
