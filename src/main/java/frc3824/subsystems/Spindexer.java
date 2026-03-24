package frc3824.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;

import frc3824.Constants.SpindexerConstants;
import frc3824.Constants.ConstantsCanIds;
import frc3824.lib.TalonFXConfig;

public class Spindexer {

    public enum SpindexerState {
        Stopped,
        Spindexing,
        Backwards
    }

    private final TalonFX m_spinnerMotor        = new TalonFX(ConstantsCanIds.SpinnerMotorId);
    private final TalonFX m_kickerMotor         = new TalonFX(ConstantsCanIds.KickerMotorId);
    private final TalonFX m_kickerFollowerMotor = new TalonFX(ConstantsCanIds.KickerFollowerMotorId);

    private SpindexerState m_state = SpindexerState.Stopped;

    private final VelocityVoltage m_spinnerRequest = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage m_kickerRequest  = new VelocityVoltage(0).withSlot(0);

    public Spindexer() {
        TalonFXConfig.configure(m_spinnerMotor,  20.0, true,  false, false, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        TalonFXConfig.configure(m_kickerMotor,   20.0, false, false, false, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    public void setState(SpindexerState newState) {
        if (newState == m_state) return;

        m_state = newState;

        double spindexerSpeed = 0.0;
        double kickerSpeed    = 0.0;

        switch (m_state) {
            case Stopped:
                break;
            case Spindexing:
                spindexerSpeed = SpindexerConstants.SpinnerWheelTurnsPerSec;
                kickerSpeed    = SpindexerConstants.KickerWheelTurnsPerSec;
                break;
            case Backwards:
                spindexerSpeed = -SpindexerConstants.SpinnerWheelTurnsPerSec;
                kickerSpeed    = -SpindexerConstants.KickerWheelTurnsPerSec;
                break;
        }

        m_spinnerMotor.setControl(m_spinnerRequest.withVelocity(spindexerSpeed));
        m_kickerMotor.setControl(m_kickerRequest.withVelocity(kickerSpeed));
    }
}