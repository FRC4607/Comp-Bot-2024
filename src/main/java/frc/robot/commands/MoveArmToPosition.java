// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToPosition extends Command {
     private final ArmSubsystem m_subsystem;
     private final double m_position;
     private final double m_tol;

     /** Creates a new MoveArmToPosition. */
     public MoveArmToPosition(double position, double tol, ArmSubsystem subsystem) {
          m_subsystem = subsystem;
          m_position = position;
          m_tol = tol;
          addRequirements(m_subsystem);
          // Use addRequirements() here to declare subsystem dependencies.
     }

     // Called when the command is initially scheduled.
     @Override
     public void initialize() {
          m_subsystem.setArmSetpoint(m_position);
     }

     // Called every time the scheduler runs while the command is scheduled.
     @Override
     public void execute() {
     }

     // Called once the command ends or is interrupted.
     @Override
     public void end(boolean interrupted) {
     }

     // Returns true when the command should end.
     @Override
     public boolean isFinished() {
          return Math.abs(m_subsystem.armPosition() - m_position) < m_tol;
     }
}
