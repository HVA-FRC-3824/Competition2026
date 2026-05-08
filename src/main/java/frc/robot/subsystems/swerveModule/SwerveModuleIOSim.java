package frc.robot.subsystems.swerveModule;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.lib.motor.io.MotorIO;
import frc.robot.lib.motor.io.TalonIO;

public class SwerveModuleIOSim implements SwerveModuleIO {

  private final SwerveModuleSimulation              m_moduleSimulation;
  private final SimulatedMotorController.GenericMotorController m_driveModel;
  private final SimulatedMotorController.GenericMotorController m_turnModel;

  private final MotorIO m_drivingMotor;
  private final MotorIO m_angleMotor;

  private final int m_num;

  public SwerveModuleIOSim(int moduleNum, SwerveModuleSimulation moduleSimulation, int driveMotorCanId, int angleMotorCanId, int angleEncoderCanId) {

    m_drivingMotor = new TalonIO(driveMotorCanId, Constants.Chassis.DriveConfig, true);
    m_angleMotor   = new TalonIO(angleMotorCanId, Constants.Chassis.TurnConfig, false);

    m_moduleSimulation = moduleSimulation;

    m_driveModel = moduleSimulation
      .useGenericMotorControllerForDrive()
      .withCurrentLimit(Amps.of(85));

    m_turnModel = moduleSimulation
      .useGenericControllerForSteer()
      .withCurrentLimit(Amps.of(40));

    m_num = moduleNum;
  }
  
  @Override
  public int getNum() {

    return m_num;
  }

  @Override
  public void setState(SwerveModuleInputs state) {
    m_angleMotor.setPosition(state.angle());

    m_turnModel.requestVoltage(m_angleMotor.getAppliedVoltage());

    m_angleMotor.simPeriodic(m_moduleSimulation.getSteerRelativeEncoderVelocity()); // Not sure if this needs to be like de-geared

    m_drivingMotor.setVelocity(state.velocity());

    m_driveModel.requestVoltage(m_drivingMotor.getAppliedVoltage());

    m_drivingMotor.simPeriodic(m_moduleSimulation.getDriveEncoderUnGearedSpeed());
  }
  
  @Override
  public void brake() {

    m_angleMotor.brake();
    m_drivingMotor.brake();
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      MetersPerSecond.of(m_moduleSimulation.getDriveWheelFinalSpeed().in(RotationsPerSecond) 
        * Constants.Chassis.WheelCircumference.in(Meters)), 
      new Rotation2d(m_moduleSimulation.getSteerRelativeEncoderPosition()));
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      Meters.of(m_moduleSimulation.getDriveWheelFinalPosition().in(Rotations) 
        * Constants.Chassis.WheelCircumference.in(Meters)), 
      new Rotation2d(m_moduleSimulation.getSteerRelativeEncoderPosition()));
  }

  @Override
  public void resetEncoders() {
    m_drivingMotor.resetEncoder(Rotations.of(0.0));
  }

  @Override
  public void setWheelAngleToForward() {
    // NOTHINGGGG
  }
  
}
