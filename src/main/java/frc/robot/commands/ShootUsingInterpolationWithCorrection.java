// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ShootUsingInterpolationWithCorrection extends SequentialCommandGroup {
    private static final double MaxSpeed = Calibrations.DrivetrainCalibrations.kSpeedAt12VoltsMps;
    private static final double MaxAngularRate = Math.PI * 2;

    private static final Rotation2d HALF_ROTATION = Rotation2d.fromDegrees(180);

    private final SwerveRequest.FieldCentricFacingAngle autoPoint = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagic)
            .withDeadband(0.1 * MaxSpeed)
            .withRotationalDeadband(0.1 * MaxAngularRate);

    /** Creates a new ShootUsingInterpolation. */
    public ShootUsingInterpolationWithCorrection(CommandSwerveDrivetrain drive, WristSubsystem wrist,
            ShooterSubsystem flywheel, KickerSubsystem kicker) {
        autoPoint.HeadingController.setPID(
                Calibrations.DrivetrainCalibrations.kHeadingPIDP,
                Calibrations.DrivetrainCalibrations.kHeadingPIDI,
                Calibrations.DrivetrainCalibrations.kHeadingPIDD);
        autoPoint.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        autoPoint.ForwardReference = ForwardReference.RedAlliance;
        this.addCommands(
                new ParallelDeadlineGroup(
                        new ParallelCommandGroup(
                                new MoveWristToPosition(() -> {
                                    return drive.getShotInfo().getWrist();
                                }, 1.0, wrist),
                                new SetShooterSpeed(() -> {
                                    return drive.getShotInfo().getSpeed();
                                }, 60, flywheel)),
                        drive.applyRequest(() -> autoPoint
                                .withTargetDirection(
                                        drive.getShotInfo().getRobot()
                                                .plus(HALF_ROTATION)
                                                .minus(drive
                                                        .getSwerveOffset())
                                                .plus(Rotation2d.fromDegrees(
                                                        SmartDashboard.getNumber("Robot Heading Offset", 0.0))))
                                .withCenterOfRotation(drive.getRotationPoint())
                                .withVelocityX(0)
                                .withVelocityY(0)
                                .withDeadband(0.1 * MaxSpeed * 0.25)
                                .withRotationalDeadband(0.1 * MaxAngularRate * 0.25)))
                        .withTimeout(0.75),
                new RunKickerWheel(3000.0, kicker).withTimeout(0.5));
    }
}
