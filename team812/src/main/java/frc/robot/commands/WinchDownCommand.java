// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WinchSubsystem;

public class WinchDownCommand extends Command {
  private final WinchSubsystem m_WinchSubsystem;

  /** Creates a new WinchCommand. */
  public WinchDownCommand(WinchSubsystem winchSubsystem) {
    m_WinchSubsystem = winchSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_WinchSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_WinchSubsystem.lowerRobot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_WinchSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
