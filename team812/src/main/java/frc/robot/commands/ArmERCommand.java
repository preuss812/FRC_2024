// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmERCommand extends CommandBase {
  /** Creates a new ArmERCommand. */
  private final ArmSubsystem m_armSubsystem;
  private final boolean m_extendRetract;

  public ArmERCommand(ArmSubsystem subsystem, boolean extendRetract) {
    m_armSubsystem = subsystem;
    m_extendRetract = extendRetract;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_extendRetract)
      m_armSubsystem.armExtend();
    else
      m_armSubsystem.armRetract();
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
