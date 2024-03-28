package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Calibrations;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ShootOverDefense extends ParallelCommandGroup {
    private final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagic)
            .withDeadband(0.1 * Calibrations.DrivetrainCalibrations.kSpeedAt12VoltsMps)
            .withRotationalDeadband(0.1 * 2 * Math.PI);
    private final static SwerveRequest.FieldCentricFacingAngle autoPoint = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagic)
            .withDeadband(0.1 * 0.1 * Calibrations.DrivetrainCalibrations.kSpeedAt12VoltsMps)
            .withRotationalDeadband(0.1 * 2 * Math.PI);
    private static final Rotation2d HALF_ROTATION = Rotation2d.fromDegrees(180);

    public ShootOverDefense(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta, ArmSubsystem arm,
            WristSubsystem wrist, KickerSubsystem kicker, ShooterSubsystem shoot,
            CommandSwerveDrivetrain drivetrain) {
        super(
                drivetrain.applyRequest(() -> {
                    if (Math.abs(theta.getAsDouble()) > 0.1) {
                        return drive.withVelocityX(-x.getAsDouble()
                                * Calibrations.DrivetrainCalibrations.kSpeedAt12VoltsMps
                                * 0.25)
                                .withVelocityY(-y.getAsDouble()
                                        * Calibrations.DrivetrainCalibrations.kSpeedAt12VoltsMps
                                        * 0.25)
                                .withDeadband(0.1
                                        * Calibrations.DrivetrainCalibrations.kSpeedAt12VoltsMps
                                        * 0.25)
                                .withRotationalDeadband(0.1 * 2 * Math.PI)
                                .withRotationalRate(
                                        -theta.getAsDouble() * 2 * Math.PI);
                    } else {
                        // https://www.chiefdelphi.com/t/is-there-a-builtin-function-to-find-the-angle-needed-to-get-one-pose2d-to-face-another-pose2d/455972/3
                        return autoPoint
                                .withTargetDirection(
                                        drivetrain.getShotInfo().getRobot()
                                                .minus(drivetrain
                                                        .getSwerveOffset()))
                                .withCenterOfRotation(drivetrain.getRotationPoint())
                                .withVelocityX(-x.getAsDouble()
                                        * Calibrations.DrivetrainCalibrations.kSpeedAt12VoltsMps
                                        * 0.25)
                                .withVelocityY(-y.getAsDouble()
                                        * Calibrations.DrivetrainCalibrations.kSpeedAt12VoltsMps
                                        * 0.25)
                                .withDeadband(0.1
                                        * Calibrations.DrivetrainCalibrations.kSpeedAt12VoltsMps
                                        * 0.25)
                                .withRotationalDeadband(0.1 * 2 * Math.PI);
                    }
                }),
                new MoveArmToPosition(() -> 90, 0.75, arm),
                new MoveWristToPosition(() -> 28.0, 0.75, wrist),
                new SetShooterSpeed(() -> 3000, 120, shoot),
                new RunCommand(() -> {
                }));
        autoPoint.HeadingController.setPID(
                Calibrations.DrivetrainCalibrations.kHeadingPIDP,
                Calibrations.DrivetrainCalibrations.kHeadingPIDI,
                Calibrations.DrivetrainCalibrations.kHeadingPIDD);
        autoPoint.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        autoPoint.ForwardReference = ForwardReference.RedAlliance;
        // new ParallelCommandGroup(
        // new MoveArmToPosition(90, 0.75, arm),
        // new MoveWristToPosition(() -> 25.0, 0.75, wrist),
        // new SetShooterSpeed(() -> 3000, 120, shoot)
        // ).withTimeout(3),
        // new RunKickerWheel(3000, kicker).withTimeout(0.5),
        // new ParallelCommandGroup(
        // new RunKickerWheel(0, kicker),
        // new SetShooterSpeed(() -> 0, 120, shoot),
        // new Retract(wrist, arm)
        // )
    }
}
