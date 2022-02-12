// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.PidConstants;;

public class ArmCommand extends CommandBase {
  /** Creates a new ArmCommand. */
  private final ArmSubsystem m_armSubsystem;
  private final double setPoint;

  public ArmCommand(ArmSubsystem subsystem, double position) {
    m_armSubsystem = subsystem;
    setPoint = position;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.setPosition(setPoint);
  }

  public boolean onTarget() {
    double error = m_armSubsystem.getPosition() - setPoint;
    if( Math.abs(error) < PidConstants.kThreshold) {
      return true;
    } else {
      return false;
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onTarget();
  }
}
