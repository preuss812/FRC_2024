/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.Constants.PidConstants;

public class DriveForwardCommand extends CommandBase {
  /**
   * Creates a new DriveForwardCommand.
   */
  private final Double m_speed;
  private final DriveTrain m_subsystem;
  private final GyroSubsystem m_gyro;
  public double targetAngle;

  public DriveForwardCommand(final DriveTrain subsystem, final GyroSubsystem gyro, final Double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_speed = speed;
    m_gyro = gyro;
    addRequirements(m_subsystem, m_gyro);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.printf("*** DriveForwardCommand m_speed: %f\n", m_speed);
    targetAngle = m_gyro.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double deltaAngle,  turningValue;
    deltaAngle = targetAngle - m_gyro.getAngle();
    turningValue = deltaAngle * PidConstants.kProportionalDriveStraight;
    if(turningValue < -1.0) {
      turningValue = -1.0;
    }
    if(turningValue > 1.0) {
      turningValue = 1.0;
    }

    System.out.printf("DriveForward Gyro angle: %f\n", m_gyro.getAngle());
    System.out.printf("DriveForward Target angle: %f\n", targetAngle);
    System.out.printf("DriveForward deltaAngle: %f\n", deltaAngle);
    System.out.printf("DriveForward turningValue: %f\n", turningValue);
    m_subsystem.drive(-m_speed, turningValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    m_subsystem.drive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
