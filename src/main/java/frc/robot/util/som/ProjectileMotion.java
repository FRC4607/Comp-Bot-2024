package frc.robot.util.som;

import edu.wpi.first.math.geometry.Translation3d;

public class ProjectileMotion {
    public static class SphericalCoordinate {
        public final double r;
        public final double theta;
        public final double phi;

        public SphericalCoordinate(Translation3d cartesian) {
            // Cartesian to spherical
            // https://en.wikipedia.org/wiki/Spherical_coordinate_system#Cartesian_coordinates
            r = cartesian.getNorm();
            theta = Math.acos(cartesian.getZ() / cartesian.getNorm());
            phi = Math.signum(cartesian.getY()) * Math.acos(cartesian.getX() / Math.sqrt(Math.pow(cartesian.getX(), 2) + Math.pow(cartesian.getY(), 2)));
        }
    }
    public static Translation3d velocityToAchive(double dX, double dY, double dZ, double vZf) {
        //System.out.println("x: " + dX + " y: " + dY + " z: " + dZ + " v: " + vZf);
        double vZi = Math.sqrt(Math.pow(vZf, 2) + 2 * 9.807 * dZ);
        double t = (vZf - vZi) / -9.807;
        double vX = dX / t;
        double vY = dY / t;
        return new Translation3d(vX, vY, vZi);
    }
}
