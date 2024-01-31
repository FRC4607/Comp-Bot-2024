// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class Retract extends Command {

  ArmSubsystem m_armSubsystem;
  ClimberSubsystem m_climberSubsystem;
  WristSubsystem m_wristSubsystem;
  /** Creates a new retract. */
  public Retract(ArmSubsystem armSubsystem, ClimberSubsystem climberSubsystem, WristSubsystem wristSubsystem) {
    m_armSubsystem = armSubsystem;
    m_climberSubsystem = climberSubsystem;
    m_wristSubsystem = wristSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSubsystem.isRunning = true;
    m_climberSubsystem.isRunning = true;
    m_wristSubsystem.isRunning = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.isRunning = false;
    m_climberSubsystem.isRunning = false;
    m_wristSubsystem.isRunning = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
