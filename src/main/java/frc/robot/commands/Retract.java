// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.WristSubsystem;

/** A command group that puts the arm and wrist in the retracted position. */
public class Retract extends SequentialCommandGroup {
    /**
     * Creates a new Retract command.
     * 
     * @param wrist A refernce to the {@link frc.robot.subsystems.WristSubsystem}
     *              object.
     * @param arm   A refernce to the {@link frc.robot.subsystems.ArmSubsystem}
     *              object.
     */
    public Retract(WristSubsystem wrist, ArmSubsystem arm) {
        super(
                new MoveArmToPosition(() -> 5, 10, arm),
                new MoveWristToPosition(() -> 95.0, 10.0, wrist),
                new MoveArmToPosition(() -> 1, 3, arm),
                new InstantCommand(() -> {
                    arm.setNeutral();
                    LEDSubsystem.setNeutral();
                }, arm));
    }
}
