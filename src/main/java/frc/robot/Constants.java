package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;

import edu.wpi.first.math.util.Units;

/**
 * A collection of static classes that hold constant values. These values are
 * not expected to change often.
 */
public final class Constants {
    /** Constants for the drivetrain. */
    public static final class DrivetrainConstants {
        /**
         * The closed-loop output type to use for the steer motors; This affects the
         * PID/FF gains for the steer motors.
         */
        public static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;
        /**
         * The closed-loop output type to use for the drive motors; This affects the
         * PID/FF gains for the drive motors.
         */
        public static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;

        /** The ratio of drive motor rotations per azimuth rotation. */
        public static final double kCoupleRatio = 3.5714285714285716;

        /** The ratio of drive motor turns to one wheel rotation. */
        public static final double kDriveGearRatio = 6.122448979591837;
        /** The ratio of steer motor turns to one azimuth rotation. */
        public static final double kSteerGearRatio = 12.8;

        /** Whether or not the steer motors on the robot should be inverted. */
        public static final boolean kSteerMotorReversed = false;
        /**
         * Whether or not the drive motors on the left side of the robot should be
         * inverted.
         */
        public static final boolean kInvertLeftSide = false;
        /**
         * Whether or not the drive motors on the right side of the robot should be
         * inverted.
         */
        public static final boolean kInvertRightSide = true;

        /**
         * The name of the CAN bus the drivetrain is on. If attached to the RoboRIO,
         * specify "rio".
         */
        public static final String kCANbusName = "kachow";
        /** The CAN ID of the Pigeon 2. */
        public static final int kPigeonID = 0;

        // Front Left
        /** The CAN ID of the front left drive motor. */
        public static final int kFrontLeftDriveMotorID = 19;
        /** The CAN ID of the front left steer motor. */
        public static final int kFrontLeftSteerMotorID = 18;
        /** The CAN ID of the front left turn encoder. */
        public static final int kFrontLeftEncoderID = 18;

        /**
         * The X position of the axis of the azimuth of the front left module from the
         * center of the robot.
         */
        public static final double kFrontLeftXPosInches = 9.75;
        /**
         * The Y position of the axis of the azimuth of the front left module from the
         * center of the robot.
         */
        public static final double kFrontLeftYPosInches = 9.75;

        // Front Right
        /** The CAN ID of the front right drive motor. */
        public static final int kFrontRightDriveMotorID = 17;
        /** The CAN ID of the front right steer motor. */
        public static final int kFrontRightSteerMotorID = 16;
        /** The CAN ID of the front right turn encoder. */
        public static final int kFrontRightEncoderID = 16;

        /**
         * The X position of the axis of the azimuth of the front right module from the
         * center of the robot.
         */
        public static final double kFrontRightXPosInches = 9.75;
        /**
         * The Y position of the axis of the azimuth of the front right module from the
         * center of the robot.
         */
        public static final double kFrontRightYPosInches = -9.75;

        // Back Left
        /** The CAN ID of the back left drive motor. */
        public static final int kBackLeftDriveMotorID = 2;
        /** The CAN ID of the back left steer motor. */
        public static final int kBackLeftSteerMotorID = 3;
        /** The CAN ID of the back left turn encoder. */
        public static final int kBackLeftEncoderID = 3;

        /**
         * The X position of the axis of the azimuth of the back left module from the
         * center of the robot.
         */
        public static final double kBackLeftXPosInches = -9.75;
        /**
         * The Y position of the axis of the azimuth of the back left module from the
         * center of the robot.
         */
        public static final double kBackLeftYPosInches = 9.75;

        // Back Right
        /** The CAN ID of the back right drive motor. */
        public static final int kBackRightDriveMotorID = 0;
        /** The CAN ID of the back right steer motor. */
        public static final int kBackRightSteerMotorID = 1;
        /** The CAN ID of the back right turn encoder. */
        public static final int kBackRightEncoderID = 1;

        /**
         * The X position of the axis of the azimuth of the back right module from the
         * center of the robot.
         */
        public static final double kBackRightXPosInches = -9.75;
        /**
         * The Y position of the axis of the azimuth of the back right module from the
         * center of the robot.
         */
        public static final double kBackRightYPosInches = -9.75;
    }

    /** Constants for the shooter. */
    public static final class ShooterConstants {
        /** The CAN ID of the outer shooter motor. */
        public static final int kOuterCANID = 44;
        /** The CAN ID of the outer shooter motor. */
        public static final int kInnerCANID = 46;

        /** Whether or not to invert the outer shooter motor. */
        public static final boolean kInvertOuter = false;
    }

    /** Constants for the kicker. */
    public static final class KickerConstants {
        /** The CAN ID of the kicker wheel motor controller. */
        public static final int kCANID = 14;
        /**
         * The gear ratio (x input rotations : 1 output rotation) of the kicker motor to
         * the kicker.
         */
        public static final double kKickerGearRatio = 3.0 * (17.0 / 20.0);
        /** The diameter of the kickers in mm. */
        public static final double kKickerDiameter = Units.inchesToMeters(1.0) * 1000;
        /** Whether or not to invert the kicker motor. */
        public static final boolean kInverted = false;
    }

    /** Constants for the intake. */
    public static final class IntakeConstants {
        /** The CAN ID of the roller motor controller. */
        public static final int kRollerCANID = 12;
        /** The CAN ID of the agitator motor controller. */
        public static final int kAgitatorCANID = 9;
        /** Whether or not to invert the roller motor. */
        public static final boolean kRollerInverted = false;
        /** Whether or not to invert the agitator motor. */
        public static final boolean kAgitatorInverted = true;
        /**
         * The gear ratio (x input rotations : 1 output rotation) of the roller motor to
         * the rollers.
         */
        public static final double kRollerGearRatio = 4.0;
        /** The diameter of the rollers in mm. */
        public static final double kRollerDiameter = 40.5;
        /**
         * The gear ratio (x input rotations : 1 output rotation) of the agitator motor
         * to the agitators.
         */
        public static final double kAgitatorGearRatio = 1.0;
        /** The diameter of the agitator wheels in mm. */
        public static final double kAgitatorDiameter = 101.6;
    }

    /** Constants for the wrist. */
    public static final class WristConstants {
        /** The CAN ID of the wrist motor controller. */
        public static final int kCANID = 15;
        /** The number of rotor rotations to produce one sensor rotation. */
        public static final double kRotorToSensor = 28.0;
        /** The number of sensor rotations to produce one mechanism rotation. */
        public static final double kSensorToMechanism = 1.0;
        /** In manual control, the top speed of the wrist in degrees per second. */
        public static final double kWristManualDegreesPerSecond = 10.0;
        /** Whether or not to invert the wrist motor. */
        public static final boolean kInverted = false;
        /** Whether or not to invert the wrist encoder. */
        public static final boolean kInvertEncoder = false;
    }

    /** Constants for the arm. */
    public static final class ArmConstants {
        /** The CAN ID of the front (closest to intake) motor in the arm gearbox. */
        public static final int kFrontCANID = 8;
        /** The CAN ID of the rear (closest to battery) motor in the arm gearbox. */
        public static final int kRearCANID = 7;
        /** Whether or not to invert both motors. */
        public static final boolean kInvertMotors = true;
        /**
         * The minimum angle (in rotations) the arm is allowed to go in reverse. SW
         * limit only.
         */
        public static final double kReverseSoftLimit = 0.0;
        /** The ratio of encoder rotations to wrist rotations. */
        public static final double kSensorToMechanism = 60.0 / 37.0;
        /** The ratio of motor rotations to encoder rotations. */
        public static final double kRotorToSensor = 60.19 / kSensorToMechanism;
        /** Whether or not to invert the encoder on the arm. */
        public static final boolean kInvertEncoder = true;
    }

    /** Constants for the LEDs */
    public static final class LEDConstants {
        /** THe CAN ID for the CANdle */
        public static final int CANdleID = 12;
        /** Total number of LEDs. */
        public static final int LedCount = 100;

    }
}
