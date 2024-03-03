package frc.robot.util.som;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class InterpolatingTreeMapShooter extends InterpolatingTreeMap<Double, ShotInfo> {

    public static ShotInfo interpolateShotInfo(ShotInfo start, ShotInfo end, double t) {
        // Linearly interpolate each value.
        return new ShotInfo(
            start.getSpeed() + (end.getSpeed() - start.getSpeed()) * Math.pow(t, 2),
            start.getWrist() + (end.getWrist() - start.getWrist()) * Math.sqrt(t)
        );
    }

    public InterpolatingTreeMapShooter() {
        super(InverseInterpolator.forDouble(), InterpolatingTreeMapShooter::interpolateShotInfo);
    }

    public static InterpolatingTreeMapShooter getShotMap() {
        InterpolatingTreeMapShooter build = new InterpolatingTreeMapShooter();
        build.put(1.6, new ShotInfo(2600, 128));
        build.put(2.0, new ShotInfo(2750, 135));
        build.put(2.5, new ShotInfo(3100, 142.5));
        build.put(3.0, new ShotInfo(3300, 145.25));
        build.put(3.5, new ShotInfo(3700, 148.75));
        build.put(4.0, new ShotInfo(4100, 152));
        build.put(4.5, new ShotInfo(4500, 154));
        build.put(5.0, new ShotInfo(4900, 156.5));
        build.put(5.5, new ShotInfo(5100, 157.5));
        build.put(6.0, new ShotInfo(5600, 158.75));
        return build;
    }
}
