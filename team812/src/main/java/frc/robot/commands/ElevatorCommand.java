/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends CommandBase {
  /*
  public ElevatorCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }
*/
private final ElevatorSubsystem m_elevatorSubsystem;
private final boolean elevatorUp;

public ElevatorCommand(ElevatorSubsystem subsystem, boolean up) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_elevatorSubsystem = subsystem;
  elevatorUp = up;
  addRequirements(subsystem);
}
  // Called just before this Command runs the first time
  @Override
  public void execute() {
    if(elevatorUp)
      m_elevatorSubsystem.up();
   else
      m_elevatorSubsystem.down();
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  m_elevatorSubsystem.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  public boolean isFinished() {
  return false;
  }
}
