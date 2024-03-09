// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class Climb extends Command {

  private ClimberSubsystem m_climber;

  private double m_speed;

  /** Creates a new Climb. */
  public Climb(double speed, ClimberSubsystem climber) {
    m_speed = speed;
    m_climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {

    m_climber.setLeftClimberSpeed(m_speed);
    m_climber.setRightClimberSpeed(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.setLeftClimberSpeed(0);
    m_climber.setRightClimberSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
