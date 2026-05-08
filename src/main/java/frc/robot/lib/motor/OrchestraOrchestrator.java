package frc.robot.lib.motor;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.RobotBase;
import lombok.Getter;

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
    
    if (m_motors.remove(id) != null) {

      m_orchestra.clearInstruments();
      m_motors.forEach((canId, motor) -> {
        m_orchestra.addInstrument(motor);
      });
    }
  }

  static public void playSong(Song song) {
    if (RobotBase.isSimulation()) return;

    StatusCode status = m_orchestra.loadMusic(song.getPath());
    Logger.recordOutput("MusicErr/load", status.toString());

    status = m_orchestra.play();
    Logger.recordOutput("MusicErr/play", status.toString());
  }

  public static enum Song {
    Poofs("254.chrp"),
    Cynthia("cynthia.chrp"),
    GymLeader("gymleader.chrp"),
    Birthday("happybirthday.chrp"),
    Pirates("POTC.chrp"),
    Song2("song2.chrp"),
    Tetris("tetris.chrp"),
    UnderTheSea("underthesea.chrp");

    @Getter private String path;

    private Song(String path) {
      this.path = path;
    }
  }
}