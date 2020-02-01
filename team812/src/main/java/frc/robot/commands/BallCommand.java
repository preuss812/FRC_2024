/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallSubsystem;

public class BallCommand extends CommandBase {
  /*
  public BallCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }
*/
private final BallSubsystem m_ballSubsystem;
private final boolean ballUp;

public BallCommand(BallSubsystem subsystem, boolean intake) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_ballSubsystem = subsystem;
  ballUp = intake;
  addRequirements(subsystem);
}
  // Called just before this Command runs the first time
  @Override
  public void execute() {
    if(ballUp)
      m_ballSubsystem.intake();
   else
      m_ballSubsystem.outtake();
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrintaketed) {
  m_ballSubsystem.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  public boolean isFinished() {
  return false;
  }
}
