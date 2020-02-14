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

public class DriveLeftInPlaceCommand extends CommandBase {
  /**
   * Creates a new DriveForwardCommand.
   */
  private final Double m_speed;
  private final DriveTrain m_subsystem;
  private final GyroSubsystem m_gyro;
  private double targetAngle;

  public DriveLeftInPlaceCommand(final DriveTrain subsystem, final GyroSubsystem gyro, final Double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_gyro = gyro;
    m_speed = speed;
    addRequirements(m_subsystem, m_gyro);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.printf("*** DriveLeftInPlaceCommand m_speed: %f\n", m_speed);
    targetAngle = (m_gyro.getAngle() - 85.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.drive(0.0, -m_speed);
    System.out.printf("Gyro angle: %f\n", m_gyro.getAngle());
    System.out.printf("Target angle: %f\n", targetAngle);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    m_subsystem.drive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_gyro.getAngle() <= targetAngle);
  }
}
