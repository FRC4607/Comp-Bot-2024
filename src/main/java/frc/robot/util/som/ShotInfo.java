package frc.robot.util.som;

public class ShotInfo {
    private final double speed;
    private final double wrist;

    /**
     * Constructs a new ShotInfo.
     * @param speed The speed of the shot, in RPM.
     * @param wrist The angle of the wrist, in degrees. 0deg points along the robot's +X axis, 90deg points along the robot's +Z axis.
     */
    public ShotInfo(double speed, double wrist) {
        this.speed = speed;
        this.wrist = wrist;
    }

    public double getSpeed() {
        return this.speed;
    }

    public double getWrist() {
        return this.wrist;
    }

    public ShotInfoWithDirection withDirection(double robot) {
        return new ShotInfoWithDirection(speed, wrist, robot);
    }
}
