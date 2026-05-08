package frc.robot.subsystems.swerveModule;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.lib.Module;
import frc.robot.lib.Module.Logged;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;

public class SwerveModule extends Module<SwerveModule.Outputs> {

  private SwerveModuleIO m_io;

  private SwerveModuleState m_desiredState = new SwerveModuleState();

  public SwerveModule(SwerveModuleIO io) {
    
    m_io = io;

    m_outputs = Outputs.zeroed();

    setName("SwerveModule" + m_io.getNum());
  }
  
  @Override
  public void updateOutputs() {

    m_outputs = new Outputs(m_io.getState(), m_desiredState, m_io.getPosition(), String.valueOf(m_io.getNum()));
  }

  public void resetSwerveModules() {

    m_io.setWheelAngleToForward();
  }

  public void drive(SwerveModuleState moduleState) {

    m_desiredState = moduleState;

    // Optimize state
    moduleState.optimize(m_io.getPosition().angle);

    // Set velocity
    AngularVelocity velocity = 
      RotationsPerSecond.of(moduleState.speedMetersPerSecond / Constants.Chassis.DriveMotorConversion);

    m_io.setState(new SwerveModuleInputs(velocity, moduleState.angle.getMeasure()));
  }

  public static record Outputs(
    SwerveModuleState measuredSwerveModuleState,
    SwerveModuleState desiredSwerveModuleState,
    SwerveModulePosition swerveModulePosition,
    String moduleName
  ) implements Logged {

    public static Outputs zeroed() {
      return new Outputs(
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModulePosition(),
        "");
    }

    @Override
    public void log() {
      Logger.recordOutput("Module " + moduleName + "/Measured drive ", measuredSwerveModuleState.speedMetersPerSecond);
      Logger.recordOutput("Module " + moduleName + "/Measured angle ", measuredSwerveModuleState.angle);
      Logger.recordOutput("Module " + moduleName + "/Desired drive ", desiredSwerveModuleState.speedMetersPerSecond);
      Logger.recordOutput("Module " + moduleName + "/Desired angle ", desiredSwerveModuleState.angle);
    }
  }
}