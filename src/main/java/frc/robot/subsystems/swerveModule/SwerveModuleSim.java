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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.lib.motor.talonFX.SimpleTalon;

public class SwerveModuleSim extends SwerveModule {

    
    private final SwerveModuleSimulation                          m_moduleSimulation;
    private final SimulatedMotorController.GenericMotorController m_driveModel;
    private final SimulatedMotorController.GenericMotorController m_turnModel;

    private final SimpleTalon m_drivingMotor;
    private final SimpleTalon m_angleMotor;

    private final int m_num;

    public SwerveModuleSim(int moduleNum, SwerveModuleSimulation moduleSimulation, int driveMotorCanId, int angleMotorCanId, int angleEncoderCanId) {

        m_inputs  = new Inputs();
        m_outputs = new Outputs();

        m_drivingMotor = new SimpleTalon(driveMotorCanId, Constants.Chassis.DriveConfig, true);
        m_angleMotor   = new SimpleTalon(angleMotorCanId, Constants.Chassis.TurnConfig, false);

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
    protected int getNum() {
        return m_num;
    }

    @Override
    protected void setPosition(Angle angle) {
        m_angleMotor.setPosition(angle);

        m_turnModel.requestVoltage(m_angleMotor.getAppliedVoltage());

        m_angleMotor.simPeriodic(m_moduleSimulation.getSteerRelativeEncoderVelocity()); // Not sure if this needs to be like de-geared
    }

    @Override
    protected void setVelocity(AngularVelocity velocity) {
        m_drivingMotor.setVelocity(velocity);

        m_driveModel.requestVoltage(m_drivingMotor.getAppliedVoltage());

        m_drivingMotor.simPeriodic(m_moduleSimulation.getDriveEncoderUnGearedSpeed());
    }

    @Override
    protected SwerveModuleState getState() {
        return new SwerveModuleState(
            MetersPerSecond.of(m_moduleSimulation.getDriveWheelFinalSpeed().in(RotationsPerSecond) 
                * Constants.Chassis.WheelCircumference), 
            new Rotation2d(m_moduleSimulation.getSteerRelativeEncoderPosition()));
    }

    @Override
    protected SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Meters.of(m_moduleSimulation.getDriveWheelFinalPosition().in(Rotations) 
                * Constants.Chassis.WheelCircumference), 
            new Rotation2d(m_moduleSimulation.getSteerRelativeEncoderPosition()));
    }

    @Override
    public void resetEncoders() {
        m_drivingMotor.resetEncoder(Rotations.of(0.0));
    }

    @Override
    public void setWheelAngleToForward(Angle forwardAngleDeg) {
        // NOTHINGGGG
    }
    
}
