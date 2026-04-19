package frc.robot.lib;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;

record Vector(
    double x,
    double y,
    double magnitude,
    Angle theta
) {
    public static Vector FromCoords(double x, double y) {
        return new Vector(
            x, y, 
            Math.sqrt(Math.pow(x,2) + Math.pow(y,2)), Radians.of(Math.atan2(x, y))
        );
    }
    
    public static Vector FromMagTheta(double magnitude, Angle theta) {
        return new Vector(
            Math.cos(theta.in(Radians)) * magnitude, Math.sin(theta.in(Radians)) * magnitude,
            magnitude, theta);
    }

    public static double dot(Vector vec1, Vector vec2) {
        return vec1.dot(vec2);
    }

    public double dot(Vector vec) {
        return (x * vec.x()) + (y * vec.y());
    }
    
    public Vector RotateBy(double theta) {
        return Vector.FromCoords(
            x * Math.cos(theta) + y * Math.sin(theta),
            y * Math.cos(theta) - x * Math.sin(theta));
    }
    
    public Vector times(double scalar) {
        return Vector.FromCoords(
            x * scalar,
            y * scalar
        );
    }
};