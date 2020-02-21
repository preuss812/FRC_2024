/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RampSubsystem;

public class RampCommand extends CommandBase {
  /**
   * Creates a new ShiftCommand.
   */

   private final RampSubsystem m_Ramp;

  public RampCommand(RampSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Ramp = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Ramp.down();
    System.out.printf("RampCommand DOWN\n");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      m_Ramp.up();
      System.out.printf("RampCommand UP\n");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
