package frc.robot.lib.motor.talonFX;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.RobotBase;

public class OrchestraOrchestrator
{
  static private Orchestra m_orchestra = new Orchestra();

  static private HashMap<Integer, TalonFX> m_motors = new HashMap<Integer,TalonFX>();

  static public void addInstrument(TalonFX motor) {
    if (RobotBase.isSimulation()) return;

    if (m_motors.put(motor.getDeviceID(), motor) == null) m_orchestra.addInstrument(motor);
  }

  static public void removeInstrument(int id) {
    if (RobotBase.isSimulation()) return;
    
    if (m_motors.remove(id) != null) 
    {
      m_orchestra.clearInstruments();
      m_motors.forEach((canId, motor) -> {
        m_orchestra.addInstrument(motor);
      });
    }
  }

  static public void playSong() {
    if (RobotBase.isSimulation()) return;

    StatusCode status = m_orchestra.loadMusic("POTC.chrp");

    if (!status.isOK()) {
      Logger.recordOutput("Music status", "IT DOESN'T WORK: " + status.getName());
    } else {
      Logger.recordOutput("Music status", "IT WORK: " + status.getName());
    }
  }
}