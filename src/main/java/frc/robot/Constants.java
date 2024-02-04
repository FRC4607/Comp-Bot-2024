package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;

/**
 * A collection of static classes that hold constant values. These values are not expected to change often.
 */
public final class Constants {
    /** Constants for the drivetrain. */
    public static final class DrivetrainConstants {
        /** The closed-loop output type to use for the steer motors; This affects the PID/FF gains for the steer motors. */
        public static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;
        /** The closed-loop output type to use for the drive motors; This affects the PID/FF gains for the drive motors. */
        public static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;

        /** The ratio of drive motor rotations per azimuth rotation. */
        public static final double kCoupleRatio = 3.5714285714285716;

        /** The ratio of drive motor turns to one wheel rotation. */
        public static final double kDriveGearRatio = 6.122448979591837;
        /** The ratio of steer motor turns to one azimuth rotation. */
        public static final double kSteerGearRatio = 12.8;
        /** The radius of the wheel attached to each swerve module. */
        public static final double kWheelRadiusInches = 2;

        /** Whether or not the steer motors on the robot should be inverted. */
        public static final boolean kSteerMotorReversed = false;
        /** Whether or not the drive motors on the left side of the robot should be inverted. */
        public static final boolean kInvertLeftSide = false;
        /** Whether or not the drive motors on the right side of the robot should be inverted. */
        public static final boolean kInvertRightSide = true;

        /** The name of hCAN bus the drivetrain is on. If attached to the RoboRIO, specify "rio". */
        public static final String kCANbusName = "kachow";
        /** The CAN Id of the Pigeon 2. */
        public static final int kPigeonId = 0;

        // Front Left
        /** The CAN Id of the front left drive motor. */
        public static final int kFrontLeftDriveMotorId = 19;
        /** The CAN Id of the front left steer motor. */
        public static final int kFrontLeftSteerMotorId = 18;
        /** The CAN Id of the front left turn encoder. */
        public static final int kFrontLeftEncoderId = 18;

        /** The X position of the axis of the azimuth of the front left module from the center of the robot. */
        public static final double kFrontLeftXPosInches = 9.75;
        /** The Y position of the axis of the azimuth of the front left module from the center of the robot. */
        public static final double kFrontLeftYPosInches = 9.75;

        // Front Right
        /** The CAN Id of the front right drive motor. */
        public static final int kFrontRightDriveMotorId = 17;
        /** The CAN Id of the front right steer motor. */
        public static final int kFrontRightSteerMotorId = 16;
        /** The CAN Id of the front right turn encoder. */
        public static final int kFrontRightEncoderId = 16;

        /** The X position of the axis of the azimuth of the front right module from the center of the robot. */
        public static final double kFrontRightXPosInches = 9.75;
        /** The Y position of the axis of the azimuth of the front right module from the center of the robot. */
        public static final double kFrontRightYPosInches = -9.75;

        // Back Left
        /** The CAN Id of the back left drive motor. */
        public static final int kBackLeftDriveMotorId = 2;
        /** The CAN Id of the back left steer motor. */
        public static final int kBackLeftSteerMotorId = 3;
        /** The CAN Id of the back left turn encoder. */
        public static final int kBackLeftEncoderId = 3;

        /** The X position of the axis of the azimuth of the back left module from the center of the robot. */
        public static final double kBackLeftXPosInches = -9.75;
        /** The Y position of the axis of the azimuth of the back left module from the center of the robot. */
        public static final double kBackLeftYPosInches = 9.75;

        // Back Right
        /** The CAN Id of the back right drive motor. */
        public static final int kBackRightDriveMotorId = 0;
        /** The CAN Id of the back right steer motor. */
        public static final int kBackRightSteerMotorId = 1;
        /** The CAN Id of the back right turn encoder. */
        public static final int kBackRightEncoderId = 1;

        /** The X position of the axis of the azimuth of the back right module from the center of the robot. */
        public static final double kBackRightXPosInches = -9.75;
        /** The Y position of the axis of the azimuth of the back right module from the center of the robot. */
        public static final double kBackRightYPosInches = -9.75;
    }

    /** Constants for the shooter. */
    public static final class ShooterConstants {
        /** The CAN Id of the upper shooter motor. */
        public static final int kUpperCANId = 5;
        /** The CAN Id of the lower shooter motor. */
        public static final int kLowerCANId = 4;

        /** Whether or not to invert the upper shooter motor. */
        public static final boolean kInvertUpper = true;
        /** Whether or not to invert the lower shooter motor. */
        public static final boolean kInvertLower = false;
    }

    /** Constants for the kicker wheel. */
    public static final class KickerWheelConstants {
        /** The CAN Id of the kicker wheel motor controller. */
        public static final int kCANId = 14;
    }

    /** Constants for the intake. */
    public static final class IntakeConstants {
        /** The CAN Id of the intake motor controller. */
        public static final int kCANId = 12;
        /** Whether or not to invert the intake motor. */
        public static final boolean kInverted = true;
    }

    /** Constants for the wrist. */
    public static final class WristConstants {
        /** The CAN Id of the wrist motor controller. */
        public static final int kCANId = 15;
        /** The number of rotor rotations to produce one sensor rotation. */
        public static final double kRotorToSensor = 1.0;
        /** The number of sensor rotations to produce one mechanism rotation. */
        public static final double kSensorToMechanism = 100.0;
        /** In manual control, the top speed of the wrist in degrees per second. */
        public static final double kWristManualDegreesPerSecond = 10.0;
    }

    /** Constants for the arm. */
    public static final class ArmConstants {
        /** The CAN Id of the front (closest to intake) motor in the arm gearbox. */
        public static final int kFrontCANId = 8;
        /** The CAN Id of the rear (closest to battery) motor in the arm gearbox. */
        public static final int kRearCANId = 7;
        /** Whether or not to invert both motors. */
        public static final boolean kInvertMotors = true;
        /** The minimum angle (in rotations) the arm is allowed to go in reverse. SW limit only. */
        public static final double kReverseSoftLimit = 0.0;
        /** The ratio of motor rotations to encoder rotations. */
        public static final double kRotorToSensor = 1.0;
        /** The ratio of encoder rotations to wrist rotations. */
        public static final double kSensorToMechanism = 60.19;
    }
}
