// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ShootUsingInterpolation extends SequentialCommandGroup {
    /** Creates a new ShootUsingInterpolation. */
    public ShootUsingInterpolation(CommandSwerveDrivetrain drive, WristSubsystem wrist, ShooterSubsystem flywheel, KickerSubsystem kicker) {
        super(
            new ParallelCommandGroup(
                new MoveWristToPosition(() -> { return drive.getShotInfo().getWrist(); }, 3.0, wrist),
                new SetShooterSpeed(() -> { return drive.getShotInfo().getSpeed(); }, 60, flywheel)
            ).withTimeout(0.5),
            new RunKickerWheel(3000.0, kicker).withTimeout(0.5)
        );
    }
}
