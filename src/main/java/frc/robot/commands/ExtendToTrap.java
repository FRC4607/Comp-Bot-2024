// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ExtendToTrap extends SequentialCommandGroup {
    /** Creates a new ExtendToTrap. */
    public ExtendToTrap(ArmSubsystem arm, WristSubsystem wrist) {
        super(new MoveArmToPosition(45, 5, arm),
                new MoveWristToPosition(() -> -90.0, 5, wrist),
                new MoveArmToPosition(85, 5, arm),
                new MoveWristToPosition(() -> -100.0, 5, wrist),
                new MoveWristToPosition(() -> -260, 5, wrist));
    }
}
