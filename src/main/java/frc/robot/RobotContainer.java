// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ManualWristControl;
import frc.robot.commands.RunKickerWheel;
import frc.robot.commands.SetShooterSpeed;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.commands.SetIntakeOpenLoop;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
public class RobotContainer {
    private static final double MaxSpeed = edu.wpi.first.math.util.Units.feetToMeters(17.3) - 0.5;
    private static final double MaxAngularRate = Math.PI;

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();

    private final IntakeSubsystem m_intake = new IntakeSubsystem();

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withDeadband(0.1 * MaxSpeed)
            .withRotationalDeadband(0.1 * MaxAngularRate);

    private final ShooterSubsystem m_shooter = new ShooterSubsystem();

    private final WristSubsystem m_wrist = new WristSubsystem();

    private final KickerSubsystem m_kicker = new KickerSubsystem();

    private void configureBindings() {
        joystick.y().whileTrue(new SetIntakeOpenLoop(() -> {
            return joystick.getRightTriggerAxis() - joystick.getLeftTriggerAxis();
        }, m_intake));
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));
        m_wrist.setDefaultCommand(new ManualWristControl(() -> {return (joystick.rightBumper().getAsBoolean() ? 1.0 : 0.0) - (joystick.leftBumper().getAsBoolean() ? 1.0 : 0.0);}, m_wrist));
        joystick.a().whileTrue(new SetShooterSpeed(5200, m_shooter));
        joystick.b().whileTrue(new RunKickerWheel(-1.0, m_kicker));
    }

    public RobotContainer() {
        configureBindings();
    }

    public Command getAutonomousCommand() {
        return new InstantCommand();
        // return AutoBuilder.buildAuto("circle_auto");
    }
}
