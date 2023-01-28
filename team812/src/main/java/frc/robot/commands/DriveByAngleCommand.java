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

public class DriveByAngleCommand extends CommandBase {
  /**
   * Creates a new DriveByAngleCommand.
   */
  private final Double m_speed;
  private final DriveTrain m_subsystem;
  private final GyroSubsystem m_gyro;
  private final double m_angle;
  private double targetAngle;

  public DriveByAngleCommand(final DriveTrain subsystem, final Double speed, final Double angle) {

    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_speed = speed;
    m_angle = angle;
    m_gyro = null;
    addRequirements(m_subsystem, m_gyro);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      // drive to a relative angle
    targetAngle = (m_gyro.getAngle() + m_angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.preussDrive(0.0, m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.preussDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return (m_gyro.getAngle() >= targetAngle);
  }
}
