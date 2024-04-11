package frc.robot.util.som;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class InterpolatingTreeMapShooter extends InterpolatingTreeMap<Double, ShotInfo> {

    public static ShotInfo interpolateShotInfo(ShotInfo start, ShotInfo end, double t) {
        // Linearly interpolate each value.
        return new ShotInfo(
            start.getSpeed() + (end.getSpeed() - start.getSpeed()) * Math.pow(t, 2),
            start.getWrist() + (end.getWrist() - start.getWrist()) * Math.pow(t, 2.0 / 3.0)
        );
    }

    public InterpolatingTreeMapShooter() {
        super(InverseInterpolator.forDouble(), InterpolatingTreeMapShooter::interpolateShotInfo);
    }

    public static InterpolatingTreeMapShooter getShotMap() {
        InterpolatingTreeMapShooter build = new InterpolatingTreeMapShooter();
        build.put(1.369, new ShotInfo(2600, 128));
        build.put(1.55, new ShotInfo(2700, 136));
        build.put(2.07, new ShotInfo(2800, 141.5));
        build.put(2.2, new ShotInfo(3000, 141));
        build.put(2.55, new ShotInfo(3100, 142.5)); // Here
        build.put(2.79, new ShotInfo(3400, 145.5)); // Here
        build.put(3.0, new ShotInfo(3800, 148)); // Here
        build.put(3.2, new ShotInfo(4000, 149.5)); // Here
        build.put(3.686, new ShotInfo(4200, 153)); // Here
        build.put(4.04, new ShotInfo(4300, 155)); // Here
        build.put(4.27, new ShotInfo(4500, 155.75)); // Here
        build.put(4.51, new ShotInfo(4700, 156.375)); // Here
        build.put(4.72, new ShotInfo(4900, 157)); // Here
        build.put(4.96, new ShotInfo(5100, 157.75)); // Here
        build.put(5.94, new ShotInfo(5600, 158.5)); // Here
        return build;
    }
}
