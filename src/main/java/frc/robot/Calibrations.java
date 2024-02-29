package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.util.Units;

/**
 * A collection of static classes used to hold calibration values. These values
 * aren't fixed and can be tuned as needed.
 */
public final class Calibrations {
    /**
     * Calibrations for the drivetrain.
     */
    public static final class DrivetrainCalibrations {
        /**
         * The gains for the steer motors of each swerve module, with gains in amps
         * and rotations.
         */
        public static final Slot0Configs kSteerGains = new Slot0Configs()
                .withKP(2000).withKI(0).withKD(25)
                .withKS(20).withKV(0).withKA(0);

        /**
         * The gains for the drive motors of each swerve module, with gains in amps and
         * rotations per second.
         */
        public static final Slot0Configs kDriveGains = new Slot0Configs()
                .withKP(8).withKI(0).withKD(0)
                .withKS(8.25).withKV(0).withKA(0);

        /**
         * The maximum current that can be applied to the drive motor of a robot locked
         * in place before the wheels start to slip.
         */
        public static final double kSlipCurrentA = 123.5;

        /** The effective radius of the wheel attached to each swerve module. */
        public static final double kWheelRadiusInches = 1.878;

        /**
         * The speed the robot would attain in meters per second if each of its drive
         * motors had 12V applied to them.
         */
        public static final double kSpeedAt12VoltsMps = 91.0 / Constants.DrivetrainConstants.kDriveGearRatio
                * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches)); // Calculates the robot's free speed from a
                                                                            // max motor speed of 91 rps.

        /** The CANcoder offset of the front left module. */
        public static final double kFrontLeftEncoderOffset = -0.150146484375;
        /** The CANcoder offset of the front right module. */
        public static final double kFrontRightEncoderOffset = -0.048095703125;
        /** The CANcoder offset of the back left module. */
        public static final double kBackLeftEncoderOffset = -0.254150390625;
        /** The CANcoder offset of the back right module. */
        public static final double kBackRightEncoderOffset = -0.037109375;

        // These are only used for simulation
        /** The simulated inertia of the steer motor of a swerve module(?). */
        public static final double kSteerInertia = 0.00001;
        /** The simulated inertia of the drive motor of a swerve module(?). */
        public static final double kDriveInertia = 0.001;

        public static final double kHeadingPIDP = 16.0;

        public static final double kHeadingPIDI = 0;

        public static final double kHeadingPIDD = 0;

        /**
         * The value A in the equation stddev = Ae^(Bx), where X is the average distance
         * to the AprilTag(s) in meters as reported by the Limelight.
         */
        public static double kVisionStdDevA = 4.7e-3;
        /**
         * The value B in the equation stddev = Ae^(Bx), where X is the average distance
         * to the AprilTag(s) in meters as reported by the Limelight.
         */
        public static double kVisionStdDevB = 0.798;
    }

    /**
     * Calibrations for the shooter.
     */
    public static final class ShooterCalibrations {
        /**
         * The kP constant of the shooter, in Amps/(rotation per second).
         */
        public static final double kP = 15.0;
        /** The kS constant of the shooter in Amps. */
        public static final double kS = 6.0;
        /**
         * The maximum current that the velocity PID should output in either direction
         * in Amps.
         */
        public static final double kMaxCurrent = 40.0;
    }

    /**
     * Calibrations for the wrist.
     */
    public static final class WristCalibrations {
        /**
         * The kP constant of the wrist motor. In units of Amps/rotation.
         */
        public static final double kP = 1400.0;
        /**
         * The kI constant of the wrist motor. In units of Amps/rotation^2.
         */
        public static final double kI = 0.0;
        /**
         * The kD constant of the wrist motor. In units of Amps/rps.
         */
        public static final double kD = 32.0;
        /** The absolute encoder offset of the wrist in rotations. */
        public static final double kEncoderOffset = -0.492919921875;
        /** The kS constant of the wrist in Amps. */
        public static final double kS = 0.25;
        /** The maximum velocity motion magic should use in rps. */
        public static final double kMotionMagicMaxVelocity = 1.0;
        /** The maximum acceleration motion magic should use in rps^2. */
        public static final double kMotionMagicMaxAcceleration = 5.0;
    }

    /** Calibrations for the arm. */
    public static final class ArmCalibrations {
        /** The gravity constant for arm feedforward in Amps. */
        public static final double kG = 20.0;
        /** The static constant for arm feedforward in Amps. */
        public static final double kS = 2.0;
        /** The proportional constant for arm feedback in A/rot. */
        public static final double kP = 1600.0;
        /** The derivative constant for arm feedback in A/rot*sec. */
        public static final double kD = 275.0;
        /** The maximum speed the arm should acheieve in rot/sec. */
        public static final double kMaxSpeedMotionMagic = 0.3;
        /** The maximum acceleration the arm should achieve in rot/sec^2. */
        public static final double kMaxAccelerationMotionMagic = 0.75;
        /** The offset of the absolute encoder on the arm in rotations. */
        public static final double kEncoderOffset = -0.32177734375;
        /**
         * The maximum amount of current the controller should be allowed to output in
         * either direction in Amps.
         */
        public static final double kMaxArmCurrentPerMotor = 40.0;
    }

    /** Calibrations for the intake. */
    public static final class IntakeCalibrations {
        /** The proportional constant for roller feedback in Amps/rps. */
        public static final double kRollerP = 10;
        /** The proportional constant for agitator feedback in duty cycle / (mm/s). */
        public static final double kAgitatorP = 0.01;
        /** The static constant for roller feedforward in Amps. */
        public static final double kRollerS = 16;
        /**
         * The derivative constant for roller feedback in Amps/(rotations per second^2).
         */
        public static final double kRollerD = 0;
        /** The maximum current the roller motor should output, in Amps. */
        public static final int kRollerMaxCurrent = 120;
    }

    /** Calibrations for the kicker. */
    public static final class KickerCalibrations {
        /** The proportional constant for roller feedback in Amps/rps. */
        public static final double kP = 15;
        /** The derivative constant for roller feedback in Amps/rps^2. */
        public static final double kD = 0;
        /** The roller feedforward constant in Amps. */
        public static final double kS = 20;
    }
}
