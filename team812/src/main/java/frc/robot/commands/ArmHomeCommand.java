// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmRotationSubsystem;

public class ArmHomeCommand extends Command {
  /** Creates a new ArmHomeCommand. */
  private final ArmRotationSubsystem m_armSubsystem;

  public ArmHomeCommand(ArmRotationSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("homearm", "starting");
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.setHomePosition(3000);
   // m_armSubsystem.setHome();
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("homearm", "end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_armSubsystem.isFwdLimitSwitchClosed()) {
      m_armSubsystem.setHome(); 
      m_armSubsystem.setPosition(m_armSubsystem.getPosition());
      return true;
    }
    return false;
  }
}
