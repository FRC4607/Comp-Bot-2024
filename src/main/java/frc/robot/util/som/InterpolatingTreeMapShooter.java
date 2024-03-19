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
        build.put(1.369, new ShotInfo(2600, 128));
        build.put(1.583, new ShotInfo(2700, 130));
        build.put(2.0, new ShotInfo(2800, 136));
        build.put(2.2, new ShotInfo(3000, 139));
        build.put(2.55, new ShotInfo(3100, 141.5));
        build.put(2.79, new ShotInfo(3400, 144.5));
        build.put(3.0, new ShotInfo(3800, 145.5));
        build.put(3.2, new ShotInfo(4000, 147.5));
        build.put(3.51, new ShotInfo(4100, 149.5));
        build.put(3.73, new ShotInfo(4200, 151));
        build.put(3.73, new ShotInfo(4200, 151));
        build.put(4.04, new ShotInfo(4300, 152.5));
        build.put(4.27, new ShotInfo(4500, 154));
        build.put(4.51, new ShotInfo(4700, 154.5));
        build.put(4.72, new ShotInfo(4900, 155.5));
        build.put(4.96, new ShotInfo(5100, 156.25));
        build.put(5.94, new ShotInfo(5600, 157.75));
        return build;
    }
}
