/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveBackwardCommand extends CommandBase {
  /**
   * Creates a new DriveForwardCommand.
   */
  private final Double m_speed;
  private final Double m_turn;
  private final DriveTrain m_subsystem;

  public DriveBackwardCommand(final DriveTrain subsystem, final Double speed, final Double turn) {

    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_speed = speed;
    m_turn = turn;
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("*** DriveBackward: initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.preussDrive(m_speed, m_turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    m_subsystem.preussDrive(0.0, 0.0);
    System.out.println("*** DriveBackward: end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
