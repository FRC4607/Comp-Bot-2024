package frc.robot.util.som;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShotInfoWithDirection extends ShotInfo {
    private final Rotation2d robot;

    /**
     * Creates a new ShotInfoWithDirection.
     * @param speed The speed of the shooter in RPM.
     * @param wrist The angle of the wrist in degrees. 0deg points towards the robot's +X axis, 90deg points towards the robot's +Z axis.
     * @param robot The angle of the robot in degrees. 0deg points towards the field's +X axis, 90deg points towards the field's +Y axis.
     */
    public ShotInfoWithDirection(double speed, double wrist, Rotation2d robot) {
        super(speed, wrist);
        this.robot = robot;
    }

    public Rotation2d getRobot() {
        return robot;
    }

    public ShotInfoWithDirection compensateRobotSpeed(double xSpeed, double ySpeed) {
        // Step 1: Spherical to Cartesian.
        // https://en.wikipedia.org/wiki/Spherical_coordinate_system#Cartesian_coordinates
        double xThis = getSpeed() * Math.sin(Math.toRadians(getWrist() - 90.0)) * robot.getCos();
        double yThis = getSpeed() * Math.sin(Math.toRadians(getWrist() - 90.0)) * robot.getSin();
        double zThis = getSpeed() * Math.cos(Math.toRadians(getWrist() - 90.0));
        // Step 2: Subtract
        double xNew = xThis - xSpeed;
        double yNew = yThis - ySpeed;
        // Step 3: Cartesian to spherical
        double r = Math.sqrt(Math.pow(xNew, 2) + Math.pow(yNew, 2) + Math.pow(zThis, 2));
        return new ShotInfoWithDirection(
            r,
            Math.toDegrees(Math.acos(zThis / r)) + 90.0,
            new Rotation2d(Math.signum(yNew) * Math.acos(xNew / Math.sqrt(Math.pow(xNew, 2) + Math.pow(yNew, 2))))
        );
    }
}
