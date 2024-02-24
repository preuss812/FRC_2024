// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TakeInNoteCommand extends Command {
  private final NoteIntakeSubsystem m_NoteIntakeSubsystem;
  private final ShooterSubsystem m_ShooterSubsystem;

  /** Creates a new TakeInNoteCommand. */
  public TakeInNoteCommand(NoteIntakeSubsystem noteIntakeSubsystem, ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_NoteIntakeSubsystem = noteIntakeSubsystem;
    m_ShooterSubsystem = shooterSubsystem;
    addRequirements(noteIntakeSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_NoteIntakeSubsystem.pickUpNote();
    m_ShooterSubsystem.shoot(0.50);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_NoteIntakeSubsystem.stop();
      m_ShooterSubsystem.shoot(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
