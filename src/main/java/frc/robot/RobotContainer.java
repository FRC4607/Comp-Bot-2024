// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Climb;
import frc.robot.commands.MoveArmToPosition;
import frc.robot.commands.MoveWristToPosition;
import frc.robot.commands.RunKickerWheel;
import frc.robot.commands.SetShooterSpeed;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.commands.RunIntakeSync;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
    private static final double MaxSpeed = edu.wpi.first.math.util.Units.feetToMeters(17.3) - 0.5;
    private static final double MaxAngularRate = Math.PI;

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);

    private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();

    private final IntakeSubsystem m_intake = new IntakeSubsystem();

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withDeadband(0.1 * MaxSpeed)
            .withRotationalDeadband(0.1 * MaxAngularRate);

    private final ShooterSubsystem m_shooter = new ShooterSubsystem();

    private final ArmSubsystem m_arm = new ArmSubsystem();

    private final WristSubsystem m_wrist = new WristSubsystem(m_arm::armPosition);

    private final KickerSubsystem m_kicker = new KickerSubsystem();

    private final ClimberSubsystem m_climber = new ClimberSubsystem();

    private void configureBindings() {
        m_kicker.setDefaultCommand(new RunIntakeSync(() -> {
            return joystick.getRightTriggerAxis() - joystick.getLeftTriggerAxis();
        }, m_intake, m_kicker));
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));
        joystick.a().whileTrue(new SetShooterSpeed(5200, m_shooter));
        joystick.b().whileTrue(new RunKickerWheel(-1.0, m_kicker));
        joystick.y().onTrue(new InstantCommand(drivetrain::seedFieldRelative, drivetrain));
        joystick.leftBumper().onTrue(new ParallelCommandGroup(new MoveArmToPosition(90.0, 5.0, m_arm), new MoveWristToPosition(30.0, 5.0, m_wrist)));
        joystick.rightBumper().onTrue(new MoveArmToPosition(5.0, 5.0, m_arm).andThen(new MoveWristToPosition(Preferences.getDouble("X Wrist", 110.0), 5.0, m_wrist)))
                .onFalse(new ParallelCommandGroup(new SetShooterSpeed(5200, m_shooter).withTimeout(2.0),
                        new WaitCommand(1.0).andThen(new RunKickerWheel(1.0, m_kicker).withTimeout(1.0))));
        joystick.povLeft().onTrue(new MoveArmToPosition(45.0, 5.0, m_arm));
        joystick.povRight().onTrue(new MoveArmToPosition(5.0, 5.0, m_arm));
        joystick.povDown().onTrue(new MoveWristToPosition(90.0, 5.0, m_wrist)
                .andThen(new MoveArmToPosition(0.0, 5.0, m_arm)).andThen(new InstantCommand(() -> {
                    m_arm.setNeutral();
                }, m_arm)));
        operatorJoystick.leftBumper().whileTrue(new Climb(0.5));
        operatorJoystick.rightBumper().whileTrue(new Climb(-0.5));
    }

    public RobotContainer() {
        Preferences.initDouble("X Wrist", 110.0);
        configureBindings();
    }

    public Command getAutonomousCommand() {
        return new InstantCommand();
        // return AutoBuilder.buildAuto("circle_auto");
    }
}
