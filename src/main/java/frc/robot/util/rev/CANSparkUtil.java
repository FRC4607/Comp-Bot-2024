package frc.robot.util.rev;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;

public class CANSparkUtil {
    public static void ConfigPIDCANSpark(double p, double i, double d, double ff, CANSparkBase m) {
        SparkPIDController c = m.getPIDController();
        c.setP(p);
        c.setI(i);
        c.setD(d);
        c.setFF(ff);
    }
}
