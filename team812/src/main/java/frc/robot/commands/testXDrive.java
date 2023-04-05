// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class testXDrive extends CommandBase {
  /** Creates a new testXDrive. */
  private double x = 0.0;
  private double y = 0.0;
  private int n = 0;
  private DriveTrain m_driveTrain;
  private int robotOrientation = 0;

  public testXDrive(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    addRequirements(driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    x = -1.0;
    y = -1.0;
    n = 0;
    robotOrientation = 0;
    m_driveTrain.setOrientation(120);
    m_driveTrain.xDrive(-1.0,0.0);
    m_driveTrain.setOrientation(robotOrientation);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.xDrive(y,x);
    if (n < 0) {
      n++;
    } else if (x < 1.0) {
      x += 0.25;
      n = 0;
    } else if (y < 1.0) {
      y += 0.25;
      x = -1.0;
      n = 0;
    } else {
      robotOrientation += 30;
      m_driveTrain.setOrientation(robotOrientation);
      n = 0;
      x=-1.0;
      y=-1.0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    x=0.0;
    y=0.1;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return robotOrientation >= 360;
  }
}
