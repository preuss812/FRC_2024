/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HookSubsystem;

public class HookCommand extends CommandBase {
  /**
   * Creates a new HookCommmand.
   */
  private final HookSubsystem m_hookSubsystem;
  private final boolean hookUp;

  public HookCommand(HookSubsystem subsystem, boolean up) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_hookSubsystem = subsystem;
    hookUp = up;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(hookUp)
      m_hookSubsystem.up();
   else
      m_hookSubsystem.down();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hookSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
