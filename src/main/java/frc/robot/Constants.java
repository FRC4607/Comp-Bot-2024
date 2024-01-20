package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;

public final class Constants {
    public static final class DrivetrainConstants {
        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        public static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        public static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        public static final double kCoupleRatio = 3.5714285714285716;

        public static final double kDriveGearRatio = 6.122448979591837;
        public static final double kSteerGearRatio = 12.8;
        public static final double kWheelRadiusInches = 2;

        public static final boolean kSteerMotorReversed = false;
        public static final boolean kInvertLeftSide = false;
        public static final boolean kInvertRightSide = true;

        public static final String kCANbusName = "kachow";
        public static final int kPigeonId = 0;

        // Front Left
        public static final int kFrontLeftDriveMotorId = 2;
        public static final int kFrontLeftSteerMotorId = 3;
        public static final int kFrontLeftEncoderId = 3;

        public static final double kFrontLeftXPosInches = 9.75;
        public static final double kFrontLeftYPosInches = 9.75;

        // Front Right
        public static final int kFrontRightDriveMotorId = 0;
        public static final int kFrontRightSteerMotorId = 1;
        public static final int kFrontRightEncoderId = 1;

        public static final double kFrontRightXPosInches = 9.75;
        public static final double kFrontRightYPosInches = -9.75;

        // Back Left
        public static final int kBackLeftDriveMotorId = 17;
        public static final int kBackLeftSteerMotorId = 16;
        public static final int kBackLeftEncoderId = 16;

        public static final double kBackLeftXPosInches = -9.75;
        public static final double kBackLeftYPosInches = 9.75;

        // Back Right
        public static final int kBackRightDriveMotorId = 19;
        public static final int kBackRightSteerMotorId = 18;
        public static final int kBackRightEncoderId = 18;

        public static final double kBackRightXPosInches = -9.75;
        public static final double kBackRightYPosInches = -9.75;
    }
}
