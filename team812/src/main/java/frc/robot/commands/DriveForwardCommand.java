/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
//import frc.robot.subsystems.GyroSubsystem;
import frc.robot.Constants.PidConstants;

import frc.robot.subsystems.EncoderSubsystem;
import frc.robot.Constants.EncoderConstants;

public class DriveForwardCommand extends CommandBase {
  /**
   * Creates a new DriveForwardCommand.
   */
  private final Double m_speed;
  private final DriveTrain m_subsystem;
  private final EncoderSubsystem m_encoder;
  //private final GyroSubsystem m_gyro;
  public double targetAngle;

  public DriveForwardCommand(final DriveTrain subsystem, final Double speed) {

//    public DriveForwardCommand(final DriveTrain subsystem, final GyroSubsystem gyro, final Double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_encoder = null;
    m_speed = speed;
    //m_gyro = gyro;
    //addRequirements(m_subsystem, m_gyro);
  }
//  public DriveForwardCommand(final DriveTrain subsystem, final GyroSubsystem gyro, final Double speed, final EncoderSubsystem encoder) {
  public DriveForwardCommand(final DriveTrain subsystem, final Double speed, final EncoderSubsystem encoder) {

    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_encoder = encoder;
    m_speed = speed;
//    m_gyro = gyro;
//    addRequirements(m_subsystem, m_gyro, m_encoder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_encoder.doReset();
    m_encoder.setDistancePerPulse(EncoderConstants.kEncoderDistanceFactor);
    System.out.println(EncoderConstants.kEncoderDistanceFactor);
//    targetAngle = m_gyro.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double deltaAngle,  turningValue;
//    deltaAngle = targetAngle - m_gyro.getAngle();
    deltaAngle = targetAngle - targetAngle; // 2022 hack until Gyro code in wpilibj is fixed
    turningValue = deltaAngle * PidConstants.kProportionalDriveStraight;
    if(turningValue < -1.0) {
      turningValue = -1.0;
    }
    if(turningValue > 1.0) {
      turningValue = 1.0;
    }
    m_subsystem.drive(-m_speed, turningValue);

    /* 
    System.out.printf("DriveForward Gyro angle: %f\n", m_gyro.getAngle());
    System.out.printf("DriveForward Target angle: %f\n", targetAngle);
    System.out.printf("DriveForward deltaAngle: %f\n", deltaAngle);
    System.out.printf("DriveForward turningValue: %f\n", turningValue);
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    m_subsystem.drive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_encoder.getNumberDist() > 100) {
      //desired distance in inches
      return true;
    }

    return false;
  }
}
