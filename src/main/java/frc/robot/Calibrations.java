package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.util.Units;

public final class Calibrations {
    public static final class DrivetrainCalibrations {
        // The following PIDFF gains are in amps and rotations (per second).
        public static final Slot0Configs kSteerGains = new Slot0Configs()
                .withKP(300).withKI(0).withKD(25)
                .withKS(2.885833333).withKV(0).withKA(0);
        public static final Slot0Configs kDriveGains = new Slot0Configs()
                .withKP(7).withKI(0).withKD(0)
                .withKS(2.447242424).withKV(0).withKA(0);

        public static final double kSlipCurrentA = 100;// Assumes a 40lb robot, g=9.81 m/s^2, Cf=0.9, and kT is 0.0192
                                                        // (ReCalc's kT value for a FOC Falcon 500)

        public static final double kSpeedAt12VoltsMps = Units.feetToMeters(17.3); // From SDS's website for a FOC Falcon
                                                                                  // with L3 gearing

        public static final double kFrontLeftEncoderOffset = 0.499267578125;
        public static final double kFrontRightEncoderOffset = 0.213134765625;
        public static final double kBackLeftEncoderOffset = -0.0498046875;
        public static final double kBackRightEncoderOffset = -0.15966796875;

        // These are only used for simulation
        public static final double kSteerInertia = 0.00001;
        public static final double kDriveInertia = 0.001;
    }
}
