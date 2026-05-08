package frc.robot.subsystems.swerveModule;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public record SwerveModuleInputs(
    AngularVelocity velocity, 
    Angle angle
) {
    
}
