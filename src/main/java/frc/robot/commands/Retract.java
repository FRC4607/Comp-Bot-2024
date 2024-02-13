// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class Retract extends SequentialCommandGroup {
  /** Creates a new retract. */
  public Retract(WristSubsystem wrist, ArmSubsystem arm) {
    super(
      new MoveArmToPosition(5.0, 5.0, arm),
      new MoveWristToPosition(90.0, 5.0, wrist)
    );
    // Use addRequirements() here to declare subsystem dependencies.
  }

  
}
