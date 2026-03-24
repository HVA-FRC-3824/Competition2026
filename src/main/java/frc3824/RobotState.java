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

    public enum IndexerState
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

    public enum IntakePosState
    {
        Stowed,
        Deployed,
        StartingPos
    }

    public enum IntakeRollerState
    {
        Intaking,
        Off,
        Backwards
    }

    public TowerState        m_towerState;
    public IndexerState    m_indexerState;
    public LedState          m_ledState;
    public IntakePosState    m_intakePosState;
    public IntakeRollerState m_intakeRollerState;

    public RobotState()
    {
        m_towerState = TowerState.Idle;
        m_indexerState = IndexerState.Stopped;
        m_ledState = LedState.MatchMode;
        m_intakePosState = IntakePosState.StartingPos;
        m_intakeRollerState = IntakeRollerState.Off;
    }

    public void Periodic()
    {
        
    }
}
