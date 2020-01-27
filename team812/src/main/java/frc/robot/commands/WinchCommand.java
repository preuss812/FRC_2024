/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WinchSubsystem;

public class WinchCommand extends CommandBase {
  /**
   * Creates a new WinchInCommand.
   */
  private final WinchSubsystem m_winchSubsystem;
  private final boolean winchIn;

  public WinchCommand(WinchSubsystem subsystem, boolean in) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_winchSubsystem = subsystem;
    winchIn = in;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(winchIn)
      m_winchSubsystem.forward();
   else
      m_winchSubsystem.reverse();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_winchSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
