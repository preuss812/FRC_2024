// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteIntakeSubsystem;
import frc.robot.Constants.ArmConstants;

public class NoteIntakeCommand extends Command {
  /** Creates a new ArmCommand. */
  private final NoteIntakeSubsystem m_armSubsystem;
  private final double m_position;
  private double setPoint;
  
  public NoteIntakeCommand(NoteIntakeSubsystem subsystem, double position) {
    m_armSubsystem = subsystem;
    m_position = position;
    System.out.println("ArmCommand class setPoint is " + m_position);
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    SmartDashboard.putString("armcmd", "started");

    setPoint = MathUtil.clamp(m_position, ArmConstants.kArmMinPosition, ArmConstants.kArmMaxPosition);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("armcmd", "end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // TODO is there a definition of finished?
  }
}
