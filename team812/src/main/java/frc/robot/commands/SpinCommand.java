/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SpinTheWheelSubsystem;

public class SpinCommand extends CommandBase {
  /**
   * Creates a new SpinCommand.
   */
  
   private final SpinTheWheelSubsystem m_SpinTheWheelSubsystem;

  public SpinCommand(SpinTheWheelSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
  m_SpinTheWheelSubsystem = subsystem;
  addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("is initialized*");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_SpinTheWheelSubsystem.m_start();
    System.out.println("is executed-");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("is ending/");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("is finished+");
    return false;
  }
}
