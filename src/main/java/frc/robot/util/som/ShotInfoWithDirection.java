package frc.robot.util.som;

public class ShotInfoWithDirection extends ShotInfo {
    private final double robot;

    /**
     * Creates a new ShotInfoWithDirection.
     * @param speed The speed of the shooter in RPM.
     * @param wrist The angle of the wrist in degrees. 0deg points towards the robot's +X axis, 90deg points towards the robot's +Z axis.
     * @param robot The angle of the robot in degrees. 0deg points towards the field's +X axis, 90deg points towards the field's +Y axis.
     */
    public ShotInfoWithDirection(double speed, double wrist, double robot) {
        super(speed, wrist);
        this.robot = robot;
    }

    public double getRobot() {
        return robot;
    }

    public ShotInfoWithDirection plus(ShotInfoWithDirection other) {
        // Step 1: Spherical to Cartesian.
        // https://en.wikipedia.org/wiki/Spherical_coordinate_system#Cartesian_coordinates
        double xThis = getSpeed() * Math.sin(Math.toRadians(getWrist())) * Math.cos(Math.toRadians(robot));
        double yThis = getSpeed() * Math.sin(Math.toRadians(getWrist())) * Math.sin(Math.toRadians(robot));
        double zThis = getSpeed() * Math.cos(Math.toRadians(getWrist()));
        double xOther = other.getSpeed() * Math.sin(Math.toRadians(other.getWrist())) * Math.cos(Math.toRadians(other.robot));
        double yOther = other.getSpeed() * Math.sin(Math.toRadians(other.getWrist())) * Math.sin(Math.toRadians(other.robot));
        double zOther = other.getSpeed() * Math.cos(Math.toRadians(other.getWrist()));
        // Step 2: Add
        double xNew = xThis + xOther;
        double yNew = yThis + yOther;
        double zNew = zThis + zOther;
        // Step 3: Cartesian to spherical
        double r = Math.sqrt(Math.pow(xNew, 2) + Math.pow(yNew, 2) + Math.pow(zNew, 2));
        return new ShotInfoWithDirection(
            r,
            Math.toDegrees(Math.acos(zNew / r)),
            Math.toDegrees(Math.signum(yNew) * Math.acos(xNew / Math.sqrt(Math.pow(xNew, 2) + Math.pow(yNew, 2))))
        );
    }
}
