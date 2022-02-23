/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorGripCommand extends CommandBase {
  /*
  public ElevatorGripCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }
*/
private final ElevatorSubsystem m_elevatorSubsystem;
private final boolean m_grip;

public ElevatorGripCommand(ElevatorSubsystem subsystem, boolean grip) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_elevatorSubsystem = subsystem;
  m_grip = grip;
  addRequirements(subsystem);
}
  // Called just before this Command runs the first time
  @Override
  public void execute() {
    if(m_grip)
      m_elevatorSubsystem.closeGrip();
   else
      m_elevatorSubsystem.openGrip();
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  public boolean isFinished() {
  return true;
  }
}
