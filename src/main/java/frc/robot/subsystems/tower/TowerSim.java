package frc.robot.subsystems.tower;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.lib.TalonFXConfig;

public class TowerSim implements TowerIO
{
    public double m_setTPS = 0.0;

    public double m_simPosTurns = 0.0;
    
    public final TalonFX         m_leaderFlywheel    = new TalonFX(Constants.CanIds.FlywheelMotorId);
    public final TalonFXSimState m_leaderFlywheelSim;;

    public FlywheelSim m_motorSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(2), 
                0.021, 
                1.0),
            DCMotor.getKrakenX60(2),
            0.2);

    public TowerSim()
    {
        m_leaderFlywheelSim = m_leaderFlywheel.getSimState();

        TalonFXConfig.configure(m_leaderFlywheel,
                85,            // Current limit
                false,            // Inverted
                false,           // Brake mode
                false, // Continuous wrap
                0.7,            // P gain
                0.0,             // I gain
                0.0,             // D gain
                0.0,             // S (static friction feedforward)
                0.19,            // V (velocity feedforward)
                0.79,            // A (acceleration feedforward)
                0.0,           // Velocity limit
                0.0);  // Acceleration limit

        m_motorSim.setInputVoltage(m_setTPS);

        m_leaderFlywheelSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    // this is called periodically so whatever, put the periodic stuff in here
    public void setSpeed(double speed)
    {
        m_setTPS = speed;
        m_leaderFlywheel.setControl(new VelocityVoltage(m_setTPS));

        // Technically periodic stuff
        m_leaderFlywheelSim.setSupplyVoltage(RobotController.getBatteryVoltage());


        m_motorSim.setInputVoltage(m_leaderFlywheelSim.getMotorVoltageMeasure().in(Volts));
        m_motorSim.update(0.02);

        m_simPosTurns += (m_motorSim.getAngularVelocityRPM() / 60.0) * 0.02;
        
        // Might need to set rotor position too but idk
        m_leaderFlywheelSim.setRawRotorPosition(m_simPosTurns);
        m_leaderFlywheelSim.setRotorVelocity(m_motorSim.getAngularVelocity().times(1));
    }

    public double getDesiredFlywheelTPS()
    {
        return m_setTPS;
    }

    public double getFlywheelTPS()
    {
        return m_motorSim.getAngularVelocityRPM() / 60.0;
    }
}
