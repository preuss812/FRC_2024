// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.Math;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class testYawDrive extends CommandBase {
  /** Creates a new testYawDrive. */
  private double x = 0.0;
  private double y = 0.0;
  private int n = 0;
  private DriveTrain m_driveTrain;
  private double robotYaw = -180.0;
  private double joystickYaw = -180.0;
  public testYawDrive(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    n = 0;
    x = 0;
    y = 0;
    robotYaw = -180.0;
    joystickYaw = -180.0;
    double joystickRadians = Math.toRadians(joystickYaw);
    x = Math.sin(joystickRadians);
    y = Math.cos(joystickRadians);

    m_driveTrain.setRobotYaw(robotYaw);
    m_driveTrain.yawDrive(-1.0,0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.yawDrive(y,x);
    if (n < 100) {
      n++; // Hold the yaw values for several iterations for better graphing on the shuffleboard.
    } else if (joystickYaw < 180.0) {
      joystickYaw += 5.0;
      double joystickRadians = Math.toRadians(joystickYaw);
      x = Math.sin(joystickRadians);
      y = Math.cos(joystickRadians);
      n=0;
    } else {
      robotYaw += 30.0;
      m_driveTrain.setRobotYaw(robotYaw);
      joystickYaw = -180.0;
      double joystickRadians = Math.toRadians(joystickYaw);
      x = Math.sin(joystickRadians);
      y = Math.cos(joystickRadians);
      n=0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return robotYaw >= 180;
  }
}
