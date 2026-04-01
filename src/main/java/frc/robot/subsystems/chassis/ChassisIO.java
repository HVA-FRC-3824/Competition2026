package frc.robot.subsystems.chassis;

import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public interface ChassisIO extends Subsystem
{
    default void driveFieldRelative(ChassisSpeeds speeds) 
    {
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);

        driveRobotRelative(DriverStation.isTeleop() ? 
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds, 
                getHeading().plus(isRed ? Rotation2d.kPi : new Rotation2d(0))) :
            speeds);
    }

    void driveRobotRelative(ChassisSpeeds speeds);

    void setModuleStates(SwerveModuleState[] desiredStates);

    default ChassisSpeeds getMeasuredSpeeds()
    {
        return Constants.Chassis.kinematics.toChassisSpeeds(getModuleStates());
    }

    SwerveModuleState[] getModuleStates();

    SwerveModulePosition[] getModulePositions();

    Pose2d getPose();

    void resetPose(Pose2d pose);

    void toggleXMode();
    boolean getIsXMode();
    
    default void resetGyroAngle()
    {
        setHeading(new Rotation2d());
    }

    default Rotation2d getHeading() {
        return getPose().getRotation();
    }

    default void setHeading(Rotation2d heading) {
        resetPose(new Pose2d(getPose().getTranslation(), heading));
    }

    default public AbstractDriveTrainSimulation getSimChassis()
    {
        return null;
    }
}