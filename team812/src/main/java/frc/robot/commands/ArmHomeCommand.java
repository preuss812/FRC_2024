// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmHomeCommand extends CommandBase {
  /** Creates a new ArmHomeCommand. */
  private final ArmSubsystem m_armSubsystem;

  public ArmHomeCommand(ArmSubsystem subsystem) {
    m_armSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("homearm", "starting");
    m_armSubsystem.setSensorPosition(10000.0);
    m_armSubsystem.setPosition(5000.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    /*
    if (m_reachedHome) {
      m_backOffCounter--;
    }
    else
    {
     
      if (m_armSubsystem.isBottomLimitSwitchClosed()) {
        m_reachedHome = true;
        m_armSubsystem.m_arm.set(0.10);
      }
      else
      {
        m_armSubsystem.m_arm.set(-0.10);
      }
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("homearm", "end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_armSubsystem.isBottomLimitSwitchClosed()) {
    //if (m_backOffCounter <= 0) {
      //m_armSubsystem.m_arm.set(0.0);
      m_armSubsystem.setSensorPosition(0.0);
      m_armSubsystem.setPosition(0.0); // Tell PID to keep arm at 0.
      return true;

    }
    else 
    {
      return false;
    }
  }
}
