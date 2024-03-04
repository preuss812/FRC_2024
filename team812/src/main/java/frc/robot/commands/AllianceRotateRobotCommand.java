// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.Utilities;

public class AllianceRotateRobotCommand extends RotateRobotCommand  {
  /** Creates a new AllianceRotateRobotCommand. */
  public AllianceRotateRobotCommand(DriveSubsystemSRX robotDrive, double theta, boolean relative) {
    super(robotDrive,Utilities.isBlueAlliance() ? theta : theta+Math.PI, relative);
  }
}
